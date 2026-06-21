#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// ==================== КОНФИГУРАЦИЯ ====================

// GPIO ПИНЫ
#define TEMP_SENSOR_PIN 4
#define HEATER_RELAY_PIN 18
#define PUMP_RELAY_PIN 19
#define VALVE_RELAY_PIN 25
#define OUTLET_PWM_PIN 23
#define ALARM_LED_PIN 21
#define STATUS_LED_PIN 22

// WiFi
#define WIFI_SSID "your_ssid"
#define WIFI_PASSWORD "your_password"

// Параметры системы
#define MAX_SENSORS 8
#define PID_UPDATE_INTERVAL 1000
#define TSARGA_STABILIZATION_CHECK 30000
#define TSARGA_STABLE_DELTA 0.3

#define DEFAULT_KP 2.0
#define DEFAULT_KI 0.5
#define DEFAULT_KD 1.0

// СКОРОСТИ ОТБОРА ФРАКЦИЙ (%)
#define DEFAULT_HEAD_SPEED 15    // Голова
#define DEFAULT_BODY_SPEED 25    // Тело
#define DEFAULT_TAILS_SPEED 8    // Хвосты

// ==================== СТРУКТУРЫ ДАННЫХ ====================

struct SensorInfo {
  DeviceAddress address;
  float temperature;
  bool active;
  char role[32];
};

struct SystemState {
  float cubeTemp;
  float cubeTarget;
  float tsargaTemp;
  float tsargaThreshold;
  bool tsargaStabilized;
  bool heaterOn;
  bool pumpOn;
  bool valveOpen;
  int outletSpeed;
  char outletMode[32];
  bool alarmActive;
};

struct OutletSpeeds {
  int head;
  int body;
  int tails;
};

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================

OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
WebServer server(80);
Preferences preferences;

SensorInfo sensorList[MAX_SENSORS];
int sensorCount = 0;

SystemState state;
OutletSpeeds outletSpeeds;

struct PIDController {
  float kp;
  float ki;
  float kd;
  float integral;
  float lastError;
  unsigned long lastTime;
};
PIDController pidController;

struct Reading {
  float cubeTemp;
  float tsargaTemp;
  unsigned long timestamp;
};
Reading readings[1000];
int readingsCount = 0;

struct TsargaMonitor {
  float lastTemp;
  unsigned long lastCheck;
  int stabilityCount;
};
TsargaMonitor tsargaMonitor;

unsigned long startTime = 0;

void initStructures() {
  state.cubeTemp = 0.0;
  state.cubeTarget = 80.0;
  state.tsargaTemp = 0.0;
  state.tsargaThreshold = 0.0;
  state.tsargaStabilized = false;
  state.heaterOn = false;
  state.pumpOn = false;
  state.valveOpen = false;
  state.outletSpeed = 0;
  strcpy(state.outletMode, "off");
  state.alarmActive = false;

  pidController.kp = DEFAULT_KP;
  pidController.ki = DEFAULT_KI;
  pidController.kd = DEFAULT_KD;
  pidController.integral = 0.0;
  pidController.lastError = 0.0;
  pidController.lastTime = 0;

  tsargaMonitor.lastTemp = 0.0;
  tsargaMonitor.lastCheck = 0;
  tsargaMonitor.stabilityCount = 0;

  outletSpeeds.head = DEFAULT_HEAD_SPEED;
  outletSpeeds.body = DEFAULT_BODY_SPEED;
  outletSpeeds.tails = DEFAULT_TAILS_SPEED;
}

void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS Mount Failed");
    return;
  }
  Serial.println("✅ SPIFFS initialized");
}

void initPreferences() {
  preferences.begin("moonshine", false);
  state.cubeTarget = preferences.getFloat("cubeTarget", 80.0);
  pidController.kp = preferences.getFloat("kp", DEFAULT_KP);
  pidController.ki = preferences.getFloat("ki", DEFAULT_KI);
  pidController.kd = preferences.getFloat("kd", DEFAULT_KD);
  
  outletSpeeds.head = preferences.getInt("headSpeed", DEFAULT_HEAD_SPEED);
  outletSpeeds.body = preferences.getInt("bodySpeed", DEFAULT_BODY_SPEED);
  outletSpeeds.tails = preferences.getInt("tailsSpeed", DEFAULT_TAILS_SPEED);
  
  Serial.printf("✅ Preferences loaded\n");
  Serial.printf("   Голова: %d%%, Тело: %d%%, Хвосты: %d%%\n", 
    outletSpeeds.head, outletSpeeds.body, outletSpeeds.tails);
}

void scanSensors() {
  sensors.begin();
  sensorCount = sensors.getDeviceCount();
  Serial.printf("🔍 Найдено датчиков: %d\n", sensorCount);
  
  for (int i = 0; i < sensorCount && i < MAX_SENSORS; i++) {
    sensors.getAddress(sensorList[i].address, i);
    sensorList[i].active = true;
    strcpy(sensorList[i].role, "unassigned");
    sensors.requestTemperaturesByAddress(sensorList[i].address);
    sensorList[i].temperature = sensors.getTempC(sensorList[i].address);
  }
}

String getSensorAddress(int index) {
  if (index >= sensorCount) return "";
  char addr[24];
  sprintf(addr, "%02X%02X%02X%02X%02X%02X%02X%02X",
    sensorList[index].address[0], sensorList[index].address[1],
    sensorList[index].address[2], sensorList[index].address[3],
    sensorList[index].address[4], sensorList[index].address[5],
    sensorList[index].address[6], sensorList[index].address[7]);
  return String(addr);
}

int findSensorByAddress(String addr) {
  for (int i = 0; i < sensorCount; i++) {
    if (getSensorAddress(i) == addr) return i;
  }
  return -1;
}

void setSensorRole(const char* role, const char* sensorAddr) {
  int idx = findSensorByAddress(String(sensorAddr));
  if (idx >= 0) {
    strcpy(sensorList[idx].role, role);
    String key = String(role);
    key += "Sensor";
    preferences.putString(key.c_str(), sensorAddr);
    Serial.printf("✅ Датчик %s назначен на роль: %s\n", sensorAddr, role);
  }
}

void readTemperatures() {
  sensors.requestTemperatures();
  for (int i = 0; i < sensorCount; i++) {
    sensorList[i].temperature = sensors.getTempC(sensorList[i].address);
    if (strcmp(sensorList[i].role, "cube") == 0) {
      state.cubeTemp = sensorList[i].temperature;
    } else if (strcmp(sensorList[i].role, "tsarga") == 0) {
      state.tsargaTemp = sensorList[i].temperature;
    }
  }
  if (readingsCount < 1000) {
    readings[readingsCount].cubeTemp = state.cubeTemp;
    readings[readingsCount].tsargaTemp = state.tsargaTemp;
    readings[readingsCount].timestamp = millis();
    readingsCount++;
  }
}

void initOutputs() {
  pinMode(HEATER_RELAY_PIN, OUTPUT);
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pinMode(VALVE_RELAY_PIN, OUTPUT);
  pinMode(ALARM_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(OUTLET_PWM_PIN, 0);
  setHeater(false);
  setPump(false);
  setValve(false);
  setOutletSpeed(0);
  digitalWrite(ALARM_LED_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
}

void setHeater(bool enable) {
  state.heaterOn = enable;
  digitalWrite(HEATER_RELAY_PIN, enable ? HIGH : LOW);
  digitalWrite(STATUS_LED_PIN, enable ? HIGH : LOW);
}

void setPump(bool enable) {
  state.pumpOn = enable;
  digitalWrite(PUMP_RELAY_PIN, enable ? HIGH : LOW);
}

void setValve(bool enable) {
  state.valveOpen = enable;
  digitalWrite(VALVE_RELAY_PIN, enable ? HIGH : LOW);
}

void setOutletSpeed(int speed) {
  speed = constrain(speed, 0, 100);
  state.outletSpeed = speed;
  uint8_t pwmValue = (speed * 255) / 100;
  ledcWrite(0, pwmValue);
  if (speed > 0) setValve(true);
  else setValve(false);
}

void checkTsargaStabilization() {
  if (state.tsargaStabilized) return;
  if (state.tsargaTemp < 30.0) return;
  unsigned long now = millis();
  if (now - tsargaMonitor.lastCheck > TSARGA_STABILIZATION_CHECK) {
    float delta = abs(state.tsargaTemp - tsargaMonitor.lastTemp);
    if (delta < TSARGA_STABLE_DELTA) {
      tsargaMonitor.stabilityCount++;
      if (tsargaMonitor.stabilityCount >= 2) {
        state.tsargaStabilized = true;
        state.tsargaThreshold = state.tsargaTemp;
        preferences.putFloat("tsargaThreshold", state.tsargaThreshold);
        Serial.printf("✅ ЦАРГА СТАБИЛИЗИРОВАНА: %.1f°C\n", state.tsargaThreshold);
      }
    } else {
      tsargaMonitor.stabilityCount = 0;
    }
    tsargaMonitor.lastTemp = state.tsargaTemp;
    tsargaMonitor.lastCheck = now;
  }
}

void controlTsargaValve() {
  if (!state.tsargaStabilized) return;
  if (state.tsargaTemp > state.tsargaThreshold + 2.0) {
    if (state.outletSpeed > 0) {
      setOutletSpeed(0);
      strcpy(state.outletMode, "auto_stop");
    }
  }
}

void resetTsargaCalibration() {
  state.tsargaStabilized = false;
  state.tsargaThreshold = 0.0;
  tsargaMonitor.lastTemp = 0.0;
  tsargaMonitor.stabilityCount = 0;
  tsargaMonitor.lastCheck = millis();
  preferences.remove("tsargaThreshold");
}

void setOutletMode(const char* mode) {
  strcpy(state.outletMode, mode);
  if (strcmp(mode, "off") == 0) setOutletSpeed(0);
  else if (strcmp(mode, "head") == 0) setOutletSpeed(outletSpeeds.head);
  else if (strcmp(mode, "body") == 0) setOutletSpeed(outletSpeeds.body);
  else if (strcmp(mode, "tails") == 0) setOutletSpeed(outletSpeeds.tails);
}

void autoSwitchOutletMode() {
  if (state.cubeTemp < 75.0) setOutletMode("off");
  else if (state.cubeTemp < 82.0 && strcmp(state.outletMode, "head") != 0) setOutletMode("head");
  else if (state.cubeTemp < 93.0 && strcmp(state.outletMode, "body") != 0) setOutletMode("body");
  else if (state.cubeTemp >= 93.0 && strcmp(state.outletMode, "tails") != 0) setOutletMode("tails");
}

float calculatePID(float current, float target) {
  unsigned long now = millis();
  unsigned long dt = now - pidController.lastTime;
  if (dt < PID_UPDATE_INTERVAL) return -1.0;
  float error = target - current;
  float pTerm = pidController.kp * error;
  pidController.integral += error * (dt / 1000.0);
  pidController.integral = constrain(pidController.integral, -100, 100);
  float iTerm = pidController.ki * pidController.integral;
  float dTerm = pidController.kd * (error - pidController.lastError) / (dt / 1000.0);
  pidController.lastError = error;
  pidController.lastTime = now;
  float output = pTerm + iTerm + dTerm;
  return constrain(output, 0, 100);
}

void controlHeater() {
  float pidOutput = calculatePID(state.cubeTemp, state.cubeTarget);
  if (pidOutput >= 0) {
    if (pidOutput > 70) {
      setHeater(true);
      setPump(false);
    } else if (pidOutput > 30) {
      setHeater(false);
      setPump(false);
    } else {
      setHeater(false);
      setPump(true);
    }
  }
}

void checkSafety() {
  if (state.cubeTemp < -100 || state.cubeTemp == DEVICE_DISCONNECTED_C) {
    setHeater(false);
    setPump(true);
    if (!state.alarmActive) {
      state.alarmActive = true;
      digitalWrite(ALARM_LED_PIN, HIGH);
    }
    return;
  }
  if (state.cubeTemp >= 105.0) {
    setHeater(false);
    setPump(true);
    setOutletSpeed(0);
    if (!state.alarmActive) {
      state.alarmActive = true;
      digitalWrite(ALARM_LED_PIN, HIGH);
    }
    return;
  }
  if (state.cubeTemp >= 100.0 && !state.alarmActive) {
    state.alarmActive = true;
    digitalWrite(ALARM_LED_PIN, HIGH);
    return;
  }
  if (state.cubeTemp < 99.0 && state.alarmActive) {
    state.alarmActive = false;
    digitalWrite(ALARM_LED_PIN, LOW);
  }
}

unsigned long lastTempRead = 0;
unsigned long lastControlCheck = 0;

void controlLoop() {
  unsigned long now = millis();
  if (now - lastTempRead >= 1000) {
    readTemperatures();
    lastTempRead = now;
  }
  checkSafety();
  if (state.alarmActive) return;
  if (now - lastControlCheck >= 1000) {
    checkTsargaStabilization();
    controlTsargaValve();
    controlHeater();
    autoSwitchOutletMode();
    lastControlCheck = now;
  }
}

void handleRoot() {
  if (SPIFFS.exists("/index.html")) {
    File file = SPIFFS.open("/index.html", "r");
    size_t size = file.size();
    std::unique_ptr<char[]> buf(new char[size]);
    file.readBytes(buf.get(), size);
    file.close();
    server.send(200, "text/html", buf.get());
  } else {
    server.send(404, "text/plain", "index.html not found");
  }
}

void handleScanSensors() {
  scanSensors();
  StaticJsonDocument<1024> json;
  JsonObject sensorsObj = json.createNestedObject("sensors");
  for (int i = 0; i < sensorCount; i++) {
    sensorsObj[getSensorAddress(i)] = sensorList[i].temperature;
  }
  String response;
  serializeJson(json, response);
  server.send(200, "application/json", response);
}

void handleSetSensorRole() {
  if (server.hasArg("role") && server.hasArg("sensor")) {
    setSensorRole(server.arg("role").c_str(), server.arg("sensor").c_str());
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing arguments");
  }
}

void handleStatus() {
  StaticJsonDocument<512> json;
  json["cube_temp"] = state.cubeTemp;
  json["cube_target"] = state.cubeTarget;
  json["tsarga_temp"] = state.tsargaTemp;
  json["tsarga_threshold"] = state.tsargaThreshold;
  json["tsarga_stabilized"] = state.tsargaStabilized;
  json["heater"] = state.heaterOn;
  json["pump"] = state.pumpOn;
  json["valve_open"] = state.valveOpen;
  json["outlet_speed"] = state.outletSpeed;
  json["outlet_mode"] = state.outletMode;
  json["alarm"] = state.alarmActive;
  json["readings_count"] = readingsCount;
  json["head_speed"] = outletSpeeds.head;
  json["body_speed"] = outletSpeeds.body;
  json["tails_speed"] = outletSpeeds.tails;
  String response;
  serializeJson(json, response);
  server.send(200, "application/json", response);
}

void handleSetCubeTarget() {
  if (server.hasArg("temp")) {
    float temp = server.arg("temp").toFloat();
    if (temp > 30 && temp < 105) {
      state.cubeTarget = temp;
      preferences.putFloat("cubeTarget", temp);
      pidController.integral = 0;
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Invalid");
}

void handleSetHeater() {
  if (server.hasArg("state")) {
    setHeater(server.arg("state") == "1");
    server.send(200, "text/plain", "OK");
  }
}

void handleSetOutletMode() {
  if (server.hasArg("mode")) {
    setOutletMode(server.arg("mode").c_str());
    server.send(200, "text/plain", "OK");
  }
}

void handleSetOutletSpeed() {
  if (server.hasArg("speed")) {
    int speed = server.arg("speed").toInt();
    setOutletSpeed(speed);
    server.send(200, "text/plain", "OK");
  }
}

void handleSetOutletSpeeds() {
  if (server.hasArg("head") && server.hasArg("body") && server.hasArg("tails")) {
    int head = server.arg("head").toInt();
    int body = server.arg("body").toInt();
    int tails = server.arg("tails").toInt();
    
    outletSpeeds.head = constrain(head, 0, 100);
    outletSpeeds.body = constrain(body, 0, 100);
    outletSpeeds.tails = constrain(tails, 0, 100);
    
    preferences.putInt("headSpeed", outletSpeeds.head);
    preferences.putInt("bodySpeed", outletSpeeds.body);
    preferences.putInt("tailsSpeed", outletSpeeds.tails);
    
    Serial.printf("✅ Скорости обновлены: Голова=%d%%, Тело=%d%%, Хвосты=%d%%\n", 
      outletSpeeds.head, outletSpeeds.body, outletSpeeds.tails);
    
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing arguments");
  }
}

void handleResetTsarga() {
  resetTsargaCalibration();
  server.send(200, "text/plain", "OK");
}

void handleHistory() {
  String json = "{\"data\":[";
  for (int i = 0; i < readingsCount; i++) {
    json += "{\"cube\":" + String(readings[i].cubeTemp, 2) + ",\"tsarga\":" + String(readings[i].tsargaTemp, 2) + ",\"time\":" + String(readings[i].timestamp) + "}";
    if (i < readingsCount - 1) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleClear() {
  readingsCount = 0;
  server.send(200, "text/plain", "OK");
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/api/scan_sensors", handleScanSensors);
  server.on("/api/set_sensor_role", handleSetSensorRole);
  server.on("/api/status", handleStatus);
  server.on("/api/set_cube_target", handleSetCubeTarget);
  server.on("/api/set_heater", handleSetHeater);
  server.on("/api/set_outlet_mode", handleSetOutletMode);
  server.on("/api/set_outlet_speed", handleSetOutletSpeed);
  server.on("/api/set_outlet_speeds", handleSetOutletSpeeds);
  server.on("/api/reset_tsarga", handleResetTsarga);
  server.on("/api/history", handleHistory);
  server.on("/api/clear", handleClear);
  server.begin();
  Serial.println("✅ Web server started");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n╔════════════════════════════════════╗");
  Serial.println("║  🥃 MOONSHINE CONTROLLER PRO 🥃  ║");
  Serial.println("╚════════════════════════════════════╝\n");
  
  initStructures();
  initOutputs();
  initSPIFFS();
  initPreferences();
  scanSensors();
  
  Serial.printf("🌐 Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✅ WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("⚠️ WiFi failed");
  }
  
  setupWebServer();
  pidController.lastTime = millis();
  startTime = millis();
  
  Serial.println("\n✅ System ready!\n");
}

void loop() {
  server.handleClient();
  controlLoop();
  delay(10);
}
