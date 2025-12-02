#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <time.h>
#include <Arduino.h>
#include <utility>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoOTA.h>


const float CRITICAL_WATER_LEVEL           = 2; // m
const float SPEED_CALC_INTERVAL            = 10 * 60 * 1000; // 10 minutes
const float MIN_NOTICABLE_LEVEL_CHANGE_M   = 0.05; // m
const float MIN_NOTICABLE_SPEED_CHANGE_MPH = 0.1; // m/h

const unsigned long MAIN_LOOP_INTERVAL             = 5000; // 5 seconds
const unsigned long WATER_STATS_PUBLISH_INTERVAL   = 10 * 60 * 1000; // 10 minutes

float previousWaterLevel = 0.0;
int readSensorCounter = 0;
float currentWaterSpeedPerHour = 0.0;

float maxRecordedWaterLevel = 0.0;
float minRecordedWaterLevel = 0.0;
float maxRecordedWaterIncreaseSpeed = 0.0;
float maxRecordedWaterDecreaseSpeed = 0.0;
String maxRecordedWaterLevelDate = "N/A";
String minRecordedWaterLevelDate = "N/A";
String maxRecordedWaterIncreaseSpeedDate = "N/A";
String maxRecordedWaterDecreaseSpeedDate = "N/A";

// --- Preferences (NVS) ---
Preferences memory;

void resetMemory();
void readMemory();

// --- WiFi ---
const char* ssid = "4M";
const char* password = "Shxt-313";
const char* host = "esp32-updater-1";

// ======================= INA219 Sensor Setup =====================
Adafruit_INA219 ina219;
#define INA219_PIN_SDA 32   // SDA
#define INA219_PIN_SCL 33   // SCL

// globalne do bufora/EMA
const float R_INTERNAL_OHM = 0.1f;   // R100
float shuntOffset_mV = 0.0f;         // będzie korektą: measured_at_4mA - expected_at_4mA
const int MEDIAN_N = 10;             // liczba próbek do mediany
float medianBuf[MEDIAN_N] = {0};     // bufor do mediany
int medianIndex = 0;

const float EMA_ALPHA = 0.2;         // współczynnik EMA (0..1)
float currentEMA_mA = 0;             // EMA prądu

// --- MQTT (HiveMQ) ---
const char* mqtt_server = "e492dd1e26cf46eb8faf8bf4c19894e2.s1.eu.hivemq.cloud"; // public HiveMQ broker
const int mqtt_port = 8883;
const char* mqtt_waterlevel_topic = "esp32/zbiornik";
const char* mqtt_waterlevel_stats_topic = "esp32/zbiornik/stats";
const char* mqtt_waterlevel_config_topic = "esp32/zbiornik/config";
const char* mqtt_user = "esp_user";
const char* mqtt_password = "Rde11#aqaa";
bool mqtt_subscribed = false;

// ===== MQTT Function prototypes =====
void mqttConnect();
void mqttReconnect();
void mqttPublishWaterLevelAndSensors(float waterLevel);
void mqttPublishWaterLevelStats();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttSubscribeConfig();

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// logger
void logger(const String &msg, const String &severity = "INFO") {
  String formatted = String("[") + severity + "] " + msg;
  Serial.println(formatted);
}

// Absolute difference helper
float absDiff(float a, float b) {
  return (a > b) ? (a - b) : (b - a);
}

// --- NTP Functions ---
String getFormattedTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "No time (NTP not working)";
  }
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

void initTimeAndWait() {
  configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
  logger("Syncing time via NTP...");

  struct tm timeinfo;
  int retry = 0;
  while (!getLocalTime(&timeinfo) && retry < 20) {
    logger("Waiting for NTP...");
    delay(1000);
    retry++;
  }

  if (getLocalTime(&timeinfo)) {
    logger("NTP time OK: " + getFormattedTime());
  } else {
    logger("NTP failed after 20s — continuing anyway", "WARN");
  }
}

void connectToWiFi() {
  // WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  logger(String("Connected to WiFi: ") + String(ssid));
  logger(String("IP Address: ") + WiFi.localIP().toString());
}

void initOTA() {
  ArduinoOTA.setHostname(host);

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      Serial.println("Start aktualizacji " + type);
    })
    .onEnd([]() {
      Serial.println("\nKoniec aktualizacji!");
    })
    .onError([](ota_error_t error) {
      Serial.printf("Blad[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  Serial.println("[INFO] OTA ready.");
}
// --- MQTT Connection ---

struct MqttData {
  float waterLevel;
  float tempSlave;
  float pressSlave;
  float tempMaster;
  float humidityMaster;
  float pressMaster;
  String date;
  String time;
  int rssi;
};

MqttData mqttData;  

void mqttConnect() {
  // Configure client and hand off connection logic to mqttReconnect()
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttReconnect();
}

void mqttSubscribeConfig() {
  if (mqttClient.subscribe(mqtt_waterlevel_config_topic, 0)) {
    logger(String("Subscribed to topic: ") + String(mqtt_waterlevel_config_topic));
    mqtt_subscribed = true;
  } else {
    logger(String("Failed to subscribe to topic: ") + String(mqtt_waterlevel_config_topic), "WARN");
    mqtt_subscribed = false;
  }
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    logger("Connecting to MQTT...");
    if (mqttClient.connect("ESP32Client_Zbiornik", mqtt_user, mqtt_password)) {
      logger("Connected to MQTT broker.");
      mqttSubscribeConfig();
    } else {
      logger(String("Failed, rc=") + String(mqttClient.state()), "WARN");
      delay(2000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = String((const char*)payload, length);

  String msg = message;
  msg.trim();
  msg.toUpperCase();

  if (String(topic) == mqtt_waterlevel_config_topic) {
    if(msg == "RESETMEMORY") {
      resetMemory();
      delay(100);
      readMemory();
      logger("Memory reset command executed.");
      return;
    }
    // parse and apply new config
    logger("Received config message: " + message);
    // tutaj można dodać obsługę konfiguracji
  } 
  else {
    logger(String("Unknown topic: ") + String(topic) + " ,message: " + message, "WARN");
    return;
  } 
}

void updateWaterLevelStats(float currentWaterLevel) {

    String now = getFormattedTime();
    String currentDate = now.substring(0, 10); // "2025-11-10"

  // level speed update every 10 minutes
    if (readSensorCounter >= (SPEED_CALC_INTERVAL / MAIN_LOOP_INTERVAL)) {
        float delta = currentWaterLevel - previousWaterLevel;
        
        // FIX: Cast SPEED_CALC_INTERVAL to a float or use a floating-point literal 
        // in the divisor (3600000.0) to ensure the result is a float (hours).
        // 3600000.0 ms = 1 hour
        const float MILLISECONDS_IN_HOUR = 3600000.0f; 
        
        float hourElapsed = ((float)SPEED_CALC_INTERVAL / MILLISECONDS_IN_HOUR) * readSensorCounter; // Multiply by readSensorCounter 
                                                // to get the total elapsed time since 
                                                // the last reading, assuming 
                                                // readSensorCounter increases 
                                                // by 1 every MAIN_LOOP_INTERVAL. 
                                                // If SPEED_CALC_INTERVAL is the total 
                                                // interval, just use the first line:
                                                 
        // Standard (and safer) calculation for the entire interval:
        float totalTimeElapsedMs = (float)SPEED_CALC_INTERVAL;
        float hourElapsedCorrect = totalTimeElapsedMs / MILLISECONDS_IN_HOUR; // hours
        
        currentWaterSpeedPerHour = (delta / hourElapsedCorrect);  // level change m/hour
        previousWaterLevel = currentWaterLevel;
        readSensorCounter = 0;

        Serial.printf("Water Speed: %.2f m/h\n", currentWaterSpeedPerHour);
        logger("DEBUG delta: " + String(delta) + " m, hoursElapsed: " + String(hourElapsedCorrect) + " h, currentWaterSpeedPerHour: " + String(currentWaterSpeedPerHour) + " m/h", "DEBUG");
    }

    if (currentWaterLevel > maxRecordedWaterLevel && absDiff(currentWaterLevel, maxRecordedWaterLevel) >= MIN_NOTICABLE_LEVEL_CHANGE_M) {
        maxRecordedWaterLevel = currentWaterLevel;
        maxRecordedWaterLevelDate = currentDate;
        memory.putString("maxWaterLevDate", maxRecordedWaterLevelDate);
    }

    if (currentWaterLevel < minRecordedWaterLevel && absDiff(currentWaterLevel, minRecordedWaterLevel) >= MIN_NOTICABLE_LEVEL_CHANGE_M) {
        minRecordedWaterLevel = currentWaterLevel;
        minRecordedWaterLevelDate = currentDate;
        memory.putFloat("minWaterLev", minRecordedWaterLevel);
        memory.putString("minWaterLevDate", minRecordedWaterLevelDate);
    }

    if (currentWaterSpeedPerHour > maxRecordedWaterIncreaseSpeed && absDiff(currentWaterSpeedPerHour, maxRecordedWaterIncreaseSpeed) >= MIN_NOTICABLE_SPEED_CHANGE_MPH) {
        maxRecordedWaterIncreaseSpeed = currentWaterSpeedPerHour;
        maxRecordedWaterIncreaseSpeedDate = currentDate;
        memory.putFloat("maxWaterPosSpd", maxRecordedWaterIncreaseSpeed);
        memory.putString("maxWaterPosSpdD", maxRecordedWaterIncreaseSpeedDate);
    }

    if (currentWaterSpeedPerHour < maxRecordedWaterDecreaseSpeed && absDiff(currentWaterSpeedPerHour, maxRecordedWaterDecreaseSpeed) >= MIN_NOTICABLE_SPEED_CHANGE_MPH) {
        maxRecordedWaterDecreaseSpeed = currentWaterSpeedPerHour;
        maxRecordedWaterDecreaseSpeedDate = currentDate;
        memory.putFloat("minWaterNegSpd", maxRecordedWaterDecreaseSpeed);
        memory.putString("minWaterNegSpdD", maxRecordedWaterDecreaseSpeedDate);
    }
}

void mqttPublishWaterLevelAndSensors(float waterLevel) {

  long rssi = WiFi.RSSI();
  String datetime = getFormattedTime();

  float percentageLevel = waterLevel / CRITICAL_WATER_LEVEL;
  updateWaterLevelStats(waterLevel);

  // create JSON
  String now = getFormattedTime();
  String date = now.substring(0, 10);   // "YYYY-MM-DD"
  String time = now.substring(11);      // "HH:MM:SS"
  String payload = "{\"waterLev\":" + String(waterLevel)  +
                   ",\"waterLevPerc\":" + String(percentageLevel) +
                   ",\"waterSpeed\":" + String(currentWaterSpeedPerHour) +
                   ",\"date\":" + String(date) +
                   ",\"time\":" + String(time) +
                   ",\"rssi\":" + String(rssi) + "}";

  // send to MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_waterlevel_topic, payload.c_str(), true);
    // logger("Send to MQTT topic: " + String(mqtt_waterlevel_topic));
    logger(String("MQTT topic: ") + String(mqtt_waterlevel_topic) + String(", Payload: ") + payload);
  } else {
    logger("MQTT not connected, trying again...");
    mqttConnect(); // function to reconnect
  }
}

void mqttPublishWaterLevelStats() {

  // create JSON
  String payload = "{"
    "\"maxLev\":\"" + String(maxRecordedWaterLevelDate) + " " + String(maxRecordedWaterLevel, 2) + "m\","
    "\"minLev\":\"" + String(minRecordedWaterLevelDate) + " " + String(minRecordedWaterLevel, 2) + "m\","
    "\"maxInc\":\"" + String(maxRecordedWaterIncreaseSpeedDate) + " " + String(maxRecordedWaterIncreaseSpeed) + "m/h\","
    "\"maxDec\":\"" + String(maxRecordedWaterDecreaseSpeedDate) + " " + String(maxRecordedWaterDecreaseSpeed) + "m/h\""
  "}";

  logger("Payload size: " + String(payload.length()) + " bytes");

  // send to MQTT
  if (mqttClient.connected()) {
    auto result = mqttClient.publish(mqtt_waterlevel_stats_topic, payload.c_str(), true);
    if(!result) {
      logger("Failed to publish water level stats result=" + String(result), "ERROR");
    }
    // logger("Send to MQTT topic: " + String(mqtt_waterlevel_topic));
    logger(String("MQTT topic: ") + String(mqtt_waterlevel_stats_topic) + String(", Payload: ") + payload);
  } else {
    logger("MQTT not connected, trying again...");
    mqttConnect(); // function to reconnect
  }
}

// --- INA219 Functions ---

void ina219_init() {
  Wire.begin(INA219_PIN_SDA, INA219_PIN_SCL);

  if (!ina219.begin()) {
    logger("Failed to find INA219 chip", "ERROR");
    while (1) { delay(10); }
  }

  ina219.setCalibration_32V_2A();
  Serial.println("INA219 initialized");
}

void calibrateShuntAtKnownCurrent(float knownCurrent_mA = 4.0f, int samples = 30, int delayMs = 30) {
  memory.begin("myApp", false); 

  float sum = 0.0f;
  for (int i = 0; i < samples; ++i) {
    float v = ina219.getShuntVoltage_mV();
    sum += v;
    delay(delayMs);
  }
  float measuredAvg_mV = sum / samples;
  float expected_mV = knownCurrent_mA * R_INTERNAL_OHM; // mA * ohm = mV
  float shuntOffset_mV = measuredAvg_mV - expected_mV;
  logger("Calibrate: measured_mV=" + String(measuredAvg_mV,6)
        + " expected_mV=" + String(expected_mV,6)
        + " offset_mV=" + String(shuntOffset_mV,6));

  memory.putFloat("shuntOffset_mV", shuntOffset_mV);
  delay(100);
}

float medianOfBuffer() {
  float tmp[MEDIAN_N];
  for (int i = 0; i < MEDIAN_N; ++i) tmp[i] = medianBuf[i];
  for (int i = 0; i < MEDIAN_N-1; ++i) for (int j = i+1; j < MEDIAN_N; ++j)
    if (tmp[j] < tmp[i]) { float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
  return tmp[MEDIAN_N/2];
}

void read_ina219() {
    readSensorCounter++;

    const float R_SHUNT_OHM = 0.1;   
    const float I_MIN = 4.0;         // 4 mA = poziom 0 m
    const float I_MAX = 20.0;        // 20 mA = poziom max (np. 4 m)

    float shunt_mV = ina219.getShuntVoltage_mV();  // mV
    float shunt_corr = shunt_mV - shuntOffset_mV;  // po korekcji offsetu

    float current_mA = shunt_corr / R_SHUNT_OHM;  

    // --- median filter ---
    medianBuf[medianIndex] = current_mA;
    medianIndex = (medianIndex + 1) % MEDIAN_N;
    float currentMed_mA = medianOfBuffer();  

    // --- EMA ---
    currentEMA_mA = EMA_ALPHA * currentMed_mA + (1 - EMA_ALPHA) * currentEMA_mA;

    // --- Water level 0–4 m ---
    float waterLevel = (currentEMA_mA - I_MIN) / (I_MAX - I_MIN) * 4.0;
    waterLevel = constrain(waterLevel, 0.0, 4.0);

    mqttData.waterLevel = waterLevel;

    // --- Logs ---
    // logger("Bus Voltage: " + String(ina219.getBusVoltage_V()) + " V");
    // logger("Shunt Voltage raw: " + String(shunt_mV) + " mV");
    // logger("Shunt Voltage corr: " + String(shunt_corr) + " mV");
    // logger("Current (mA, raw): " + String(current_mA));
    // logger("Current (mA, median): " + String(currentMed_mA));
    // logger("Current (mA, EMA): " + String(currentEMA_mA));
    logger("Water Level: " + String(waterLevel) + " m");

    mqttPublishWaterLevelAndSensors(waterLevel);
}

void resetMemory() {
  logger("Resetting stored statistics in NVS...");
  memory.begin("myApp", false);
  memory.clear();
  delay(10);
  memory.putFloat("minWaterLev", 4.0); // start high
  memory.end();
  memory.begin("myApp", false);

  // restore calibrated shunt offset
  memory.putFloat("shuntOffset_mV", shuntOffset_mV);
}

void readMemory()
{
  logger("Reading stored statistics from NVS...");

  memory.begin("myApp", false); 

  maxRecordedWaterLevel = memory.getFloat("maxWaterLev", 0.0);
  minRecordedWaterLevel = memory.getFloat("minWaterLev", 0.0);
  maxRecordedWaterLevelDate = memory.getString("maxWaterLevDate", "N/A");
  minRecordedWaterLevelDate = memory.getString("minWaterLevDate", "N/A");
  maxRecordedWaterIncreaseSpeed = memory.getFloat("maxWaterPosSpd", 0.0);
  maxRecordedWaterDecreaseSpeed = memory.getFloat("minWaterNegSpd", 0.0);
  maxRecordedWaterIncreaseSpeedDate = memory.getString("maxWaterPosSpdD", "N/A");
  maxRecordedWaterDecreaseSpeedDate = memory.getString("minWaterNegSpdD", "N/A");
  shuntOffset_mV = memory.getFloat("shuntOffset_mV", 0.0);

  // Log all values separately at the end of the method
  logger("maxRecordedWaterLevel: " + String(maxRecordedWaterLevel, 6));
  logger("minRecordedWaterLevel: " + String(minRecordedWaterLevel, 6));
  logger("maxRecordedWaterLevelDate: " + maxRecordedWaterLevelDate);
  logger("minRecordedWaterLevelDate: " + minRecordedWaterLevelDate);
  logger("maxRecordedWaterIncreaseSpeed: " + String(maxRecordedWaterIncreaseSpeed, 6));
  logger("maxRecordedWaterDecreaseSpeed: " + String(maxRecordedWaterDecreaseSpeed, 6));
  logger("maxRecordedWaterIncreaseSpeedDate: " + maxRecordedWaterIncreaseSpeedDate);
  logger("maxRecordedWaterDecreaseSpeedDate: " + maxRecordedWaterDecreaseSpeedDate);
  logger("shuntOffset_mV: " + String(shuntOffset_mV, 6));
}

void setup() {
  logger("Initializing...");
  espClient.setInsecure(); // disables certificate verification

  Serial.begin(115200);
  connectToWiFi();
  initOTA();
  initTimeAndWait();

  mqttConnect();

  ina219_init();
  // calibrateShuntAtKnownCurrent();

  // just to use mqttClient and avoid
  logger("Mqtt buffer size: " + String(mqttClient.getBufferSize()) + " bytes");
  readMemory();
}

void loop() {

  ArduinoOTA.handle();

  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  if(WiFi.status() != WL_CONNECTED) {
    logger("WiFi disconnected, reconnecting...");
    connectToWiFi();
  }

  if (mqttClient.state() != 0) {
    logger("MQTT internal state changed: " + String(mqttClient.state()));
  }

  unsigned long nowMs = millis();

  // read INA219 sensor periodically
  static unsigned long lastRead = 0;
  if (nowMs - lastRead >= (unsigned long)MAIN_LOOP_INTERVAL) {
    lastRead = nowMs;
    read_ina219();
  }

  // publish water level stats periodically
  static unsigned long lastPublishMs = 0;
  if (lastPublishMs == 0) lastPublishMs = nowMs;

  if (nowMs - lastPublishMs >= (unsigned long)WATER_STATS_PUBLISH_INTERVAL) {
    mqttPublishWaterLevelStats();
    lastPublishMs = nowMs;
  }


}