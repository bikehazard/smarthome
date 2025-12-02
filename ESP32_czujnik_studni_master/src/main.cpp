#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <time.h>
#include <Arduino.h>
#include <utility>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include <tuple>
#include <Preferences.h>
#include <ArduinoOTA.h>

unsigned long lastLoop = 0;
const unsigned long READ_SENSOR_INTERVAL   = 10000; // 10 seconds
const float SPEED_CALC_INTERVAL            = 10 * 60 * 1000; // 10 minutes
const float CRITICAL_WATER_LEVEL           = 12.5; // m
const float MIN_NOTICABLE_LEVEL_CHANGE_M   = 0.05; // m
const float MIN_NOTICABLE_SPEED_CHANGE_MPM = 0.01; // m/min
const float WATER_STATS_PUBLISH_INTERVAL   = 10 * 60 * 1000; // 10 minutes

float previousWaterLevel = 0.0;
int readSlaveSensorCounter = 0;
float currentWaterSpeedPerHour = 0.0;

float maxRecordedWaterLevel = 0.0;
float minRecordedWaterLevel = 0.0;
float maxRecordedWaterIncreaseSpeed = 0.0;
float maxRecordedWaterDecreaseSpeed = 0.0;
float waterLevel12hAgo = 0.0;
float waterLevel1hAgo = 0.0;
String maxRecordedWaterLevelDate = "N/A";
String minRecordedWaterLevelDate = "N/A";
String maxRecordedWaterIncreaseSpeedDate = "N/A";
String maxRecordedWaterDecreaseSpeedDate = "N/A";  


// Preferences for storing stats

Preferences memory;
void resetMemory();
void readMemory();

// ======================= UART (HardwareSerial) =====================
#define RX_PIN 18 // RO
#define TX_PIN 16 // DI
#define RE_DE_PIN 5
#define UART_BAUD 9600
HardwareSerial uartToSlaveESP(1);

// ======================= BMP280 Sensor Setup =====================
#define BMP280_PIN_SDA 22   // SDA
#define BMP280_PIN_SCL 23   // SCL

Adafruit_BMP280 bmp280;
Adafruit_AHTX0 aht20;
bool bmp_ok = false;
bool aht_ok = false;

// --- WiFi ---
// const char* ssid = "4M";
// const char* password = "Shxt-313";
const char* ssid = "Mnet_0";
const char* password = "Vgt15rst";
const char* host = "esp32-updater-studnia";

// --- MQTT (HiveMQ) ---
const char* mqtt_server = "e492dd1e26cf46eb8faf8bf4c19894e2.s1.eu.hivemq.cloud"; // public HiveMQ broker
const int mqtt_port = 8883;
const char* mqtt_waterlevel_topic = "esp32/studnia";
const char* mqtt_logs_topic = "esp32/studnia/logs";
const char* mqtt_esp32_config_topic = "esp32/studnia/config";  // where broker sends config
const char* mqtt_waterlevel_stats_topic = "esp32/studnia/stats";
const char* mqtt_user = "esp_user";
const char* mqtt_password = "Rde11#aqaa";
bool mqtt_subscribed = false;
// forward logs to MQTT when true
bool mqtt_enable_master_logs = false;
bool mqtt_enable_slave_logs = false;

// ===== MQTT Function prototypes =====
void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttConnect();
void mqttReconnect();
void mqttSubscribeConfig();
void mqttPublishWaterLevelAndSensors(float tempValue_slave, float pressValue_slave, float tempValue_master, float humidityValue_master, float pressValue_master);
void mqttPublishWaterLevelStats();

// --- NTP Functions ---
void initTime() {
  configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
}

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

String getFormattedTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "No time (NTP not working)";
  }
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

// Logging wrapper: print to Serial and optionally forward to MQTT (master logs)
// Adds an optional severity prefix at the beginning of the log string: [SEVERITY]
void logger(const String &msg, const String &severity = "INFO") {
  String formatted = String("[") + severity + "] " + msg;
  Serial.println(formatted);
  if (mqttClient.connected() && mqtt_enable_master_logs) {
    String now = getFormattedTime();
    // Build JSON payload with time and log fields
    String payload = String("{\"time\":\"") + now + String("\",\"log\":\"") + formatted + String("\"}");
    mqttClient.publish(mqtt_logs_topic, payload.c_str());
  }
}

void scanWifi() {
  int n = WiFi.scanNetworks();
  logger("Available WiFi networks:");
  for (int i = 0; i < n; ++i) {
    String line = String(i + 1) + ": " + String(WiFi.SSID(i)) + " (RSSI: " + String(WiFi.RSSI(i)) + " dBm) " + ((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Secured");
    logger(line);
  }
  if (n == 0) {
    logger("No available WiFi networks");
  }
}

void connectToWiFi() {
  // WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    // periodically scan for networks but avoid noisy dot printing
    scanWifi();
  }

  logger(String("Connected to WiFi: ") + String(ssid));
  logger(String("IP Address: ") + WiFi.localIP().toString());
}

// --- MQTT Connection ---

void mqttCallback(char* topic, byte* payload, unsigned int length) {

  // topic is a null-terminated string; payload may NOT be null-terminated.
  // Safely construct a String from the payload using the provided length.
  String message = String((const char*)payload, length);
  logger("[MQTT] Callback invoked. Topic: " + String(topic) + " ,message: " + String(message));

  // --- Process the config string here ---
  // if (String(topic) == mqtt_esp32_config_topic) {
    String msg = message;
    msg.trim();
    msg.toUpperCase();

    if (msg.startsWith("{")) {
      // TODO: parse JSON configuration if needed
    } else if (msg.startsWith("DEBUG=")) {
      String val = msg.substring(msg.indexOf('=') + 1);
      val.trim();
      if (val == "1") {
        mqtt_enable_master_logs = true;
        mqtt_enable_slave_logs = true;
        logger("Config: DEBUG=1 -> enabling log forwarding to MQTT");
      } else if (val == "0") {
        mqtt_enable_master_logs = false;
        mqtt_enable_slave_logs = false;
        logger("Config: DEBUG=0 -> disabling log forwarding to MQTT");
      } else {
        logger(String("Unknown DEBUG value: ") + val, "WARN");
      }
    } 
    else if (msg == "RESET") {
      logger("Config: RESET received -> rebooting ESP32...", "WARN");
      delay(2000);
      ESP.restart();
      delay(2000);
    } else if (msg == "RESETMEMORY")  {
      resetMemory();
      delay(100);
      readMemory();
      logger("Memory reset command executed.");
      return;
    } 
    else {
      logger(String("Received unknown config: ") + message);
      // For example, save to variable or preferences
    }
  // }
  // else {
  //   logger(String("Unknown topic: ") + String(topic) + " ,message: " + String(message), "WARN");
  // }
}

void mqttConnect() {
  // Configure client and hand off connection logic to mqttReconnect()
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttReconnect();
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    logger("Connecting to MQTT...");
    if (mqttClient.connect("ESP32Client_Studnia_Master", mqtt_user, mqtt_password)) {
      logger("connected");
      // ensure subscription is attempted after a successful connect
      mqttSubscribeConfig();
    } else {
      logger(String("failed, rc=") + String(mqttClient.state()), "WARN");
      delay(2000);
    }
  }
}

float calculateWaterLevel(float pressure_hpa_slave, float pressure_hpa_master) {
  // Assuming pressure is in mbar and the density of water is 1000 kg/m^3
  // Water level (in meters) = Pressure (in mbar) / (density of water * g) * 100
  // where g is the acceleration due to gravity (approximately 9.81 m/s^2)

  float rho = 1000.0; // density of water in kg/m^3
  float g = 9.80665; // acceleration due to gravity in m/s^2

  float offset = 10.64; // calibration offset in hPa between sensors

  float water_level_m = (pressure_hpa_slave - pressure_hpa_master + offset) * 100 / (rho * g); // convert pressure to pa

  // logger(String("Calculated Water Level: ") + String(water_level_m) + " m");
  return water_level_m;
}

void publishLogToMQTT(const String &logMessage) {
  if (!mqtt_enable_slave_logs) {
    // Skip forwarding logs 
    return;
  }

  String now = getFormattedTime();
  String fullMessage = now + " | " + logMessage;
  String payload = "{\"log\":\"" + fullMessage + "\"}";

  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_logs_topic, payload.c_str());
    // keep local-only prints here to avoid double-forwarding via logger
    Serial.println("Sent log to MQTT topic: " + String(mqtt_logs_topic));
    Serial.println("Log Payload: " + payload);
  } else {
    Serial.println("MQTT not connected, trying again...");
    mqttConnect(); // function to reconnect
  }
}

float absDiff(float a, float b) {
    return (a > b) ? (a - b) : (b - a);
}

void updateWaterLevelStats(float currentWaterLevel) {

    String now = getFormattedTime();
    String currentDate = now.substring(0, 10); // "2025-11-10"

    if (readSlaveSensorCounter >= (SPEED_CALC_INTERVAL / READ_SENSOR_INTERVAL)) {
        float delta = currentWaterLevel - previousWaterLevel;
        
        // FIX: Cast SPEED_CALC_INTERVAL to a float or use a floating-point literal 
        // in the divisor (3600000.0) to ensure the result is a float (hours).
        // 3600000.0 ms = 1 hour
        const float MILLISECONDS_IN_HOUR = 3600000.0f; 
        
        float hourElapsed = ((float)SPEED_CALC_INTERVAL / MILLISECONDS_IN_HOUR) * readSlaveSensorCounter; // Multiply by readSlaveSensorCounter 
                                                // to get the total elapsed time since 
                                                // the last reading, assuming 
                                                // readSlaveSensorCounter increases 
                                                // by 1 every READ_SENSOR_INTERVAL. 
                                                // If SPEED_CALC_INTERVAL is the total 
                                                // interval, just use the first line:
                                                 
        // Standard (and safer) calculation for the entire interval:
        float totalTimeElapsedMs = (float)SPEED_CALC_INTERVAL;
        float hourElapsedCorrect = totalTimeElapsedMs / MILLISECONDS_IN_HOUR; // hours
        
        currentWaterSpeedPerHour = (delta / hourElapsedCorrect);  // level change m/hour
        previousWaterLevel = currentWaterLevel;
        readSlaveSensorCounter = 0;

        Serial.printf("Water Speed: %.2f m/h\n", currentWaterSpeedPerHour);
        logger("DEBUG delta: " + String(delta) + " m, hoursElapsed: " + String(hourElapsedCorrect) + " h, currentWaterSpeedPerHour: " + String(currentWaterSpeedPerHour) + " m/h", "DEBUG");
    }

    if (currentWaterLevel > maxRecordedWaterLevel && absDiff(currentWaterLevel, maxRecordedWaterLevel) >= MIN_NOTICABLE_LEVEL_CHANGE_M) {
        maxRecordedWaterLevel = currentWaterLevel;
        maxRecordedWaterLevelDate = currentDate;
        memory.putFloat("maxWaterLev", maxRecordedWaterLevel);
        memory.putString("maxWaterLevDate", maxRecordedWaterLevelDate);
    }

    if (currentWaterLevel < minRecordedWaterLevel && absDiff(currentWaterLevel, minRecordedWaterLevel) >= MIN_NOTICABLE_LEVEL_CHANGE_M) {
        minRecordedWaterLevel = currentWaterLevel;
        minRecordedWaterLevelDate = currentDate;
        memory.putFloat("minWaterLev", minRecordedWaterLevel);
        memory.putString("minWaterLevDate", minRecordedWaterLevelDate);
    }

    if (currentWaterSpeedPerHour > maxRecordedWaterIncreaseSpeed && absDiff(currentWaterSpeedPerHour, maxRecordedWaterIncreaseSpeed) >= MIN_NOTICABLE_SPEED_CHANGE_MPM) {
        maxRecordedWaterIncreaseSpeed = currentWaterSpeedPerHour;
        maxRecordedWaterIncreaseSpeedDate = currentDate;
        memory.putFloat("maxWaterPosSpd", maxRecordedWaterIncreaseSpeed);
        memory.putString("maxWaterPosSpdD", maxRecordedWaterIncreaseSpeedDate);
    }

    if (currentWaterSpeedPerHour < maxRecordedWaterDecreaseSpeed && absDiff(currentWaterSpeedPerHour, maxRecordedWaterDecreaseSpeed) >= MIN_NOTICABLE_SPEED_CHANGE_MPM) {
        maxRecordedWaterDecreaseSpeed = currentWaterSpeedPerHour;
        maxRecordedWaterDecreaseSpeedDate = currentDate;
        memory.putFloat("minWaterNegSpd", maxRecordedWaterDecreaseSpeed);
        memory.putString("minWaterNegSpdD", maxRecordedWaterDecreaseSpeedDate);
    }
}

void mqttPublishWaterLevelAndSensors(float tempValue_slave, float pressValue_slave, float tempValue_master, float humidityValue_master, float pressValue_master) {
  
  float waterLevel = calculateWaterLevel(pressValue_slave, pressValue_master);

  long rssi = WiFi.RSSI();
  String datetime = getFormattedTime();

  float percentageLevel = waterLevel / CRITICAL_WATER_LEVEL;
  updateWaterLevelStats(waterLevel);

  // create JSON
  String now = getFormattedTime();
  String date = now.substring(0, 10);   // "YYYY-MM-DD"
  String time = now.substring(11);      // "HH:MM:SS"
  String payload = "{\"waterLev\":" + String(waterLevel)  +
                   ",\"waterLevPerc\":\"" + String(percentageLevel) +
                   ",\"waterSpeed\":" + String(currentWaterSpeedPerHour) +
                   ",\"tempSlave\":" + String(tempValue_slave) +
                   ",\"pressSlave\":" + String(pressValue_slave) +
                   ",\"tempMaster\":" + String(tempValue_master) +
                   ",\"humidityMaster\":" + String(humidityValue_master) +
                   ",\"pressMaster\":" + String(pressValue_master) +
                   ",\"date\":\"" + String(date) +
                   ",\"time\":\"" + String(time) +
                   ",\"rssi\":" + String(rssi) + "}";

  // send to MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_waterlevel_topic, payload.c_str());
    // logger("Send to MQTT topic: " + String(mqtt_waterlevel_topic));
    logger("MQTT Payload: " + payload);
  } else {
    logger("MQTT not connected, trying again...");
    mqttConnect(); // function to reconnect
  }
}

void mqttPublishWaterLevelStats() {

  long rssi = WiFi.RSSI();
  String datetime = getFormattedTime();

  // create JSON
  String now = getFormattedTime();
  String date = now.substring(0, 10);   // "YYYY-MM-DD"
  String time = now.substring(11);      // "HH:MM:SS"

  String payload = "{"
    "\"maxLev\":\"" + String(maxRecordedWaterLevelDate) + " " + String(maxRecordedWaterLevel, 2) + "m\","
    "\"minLev\":\"" + String(minRecordedWaterLevelDate) + " " + String(minRecordedWaterLevel, 2) + "m\","
    "\"maxInc\":\"" + String(maxRecordedWaterIncreaseSpeedDate) + " " + String(maxRecordedWaterIncreaseSpeed) + "m/h\","
    "\"maxDec\":\"" + String(maxRecordedWaterDecreaseSpeedDate) + " " + String(maxRecordedWaterDecreaseSpeed) + "m/h\","
    "\"date\":\"" + String(date) + "\","
    "\"time\":\"" + String(time) + "\","
    "\"rssi\":" + String(rssi) + "\""
  "}";

  // send to MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_waterlevel_stats_topic, payload.c_str());
    // logger("Send to MQTT topic: " + String(mqtt_waterlevel_topic));
    logger("MQTT Payload: " + payload);
  } else {
    logger("MQTT not connected, trying again...");
    mqttConnect(); // function to reconnect
  }
}

void printPendingSlaveMessages() {
  while (uartToSlaveESP.available()) {
    String line = uartToSlaveESP.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
        Serial.println("[SLAVE] " + line);
        publishLogToMQTT("[SLAVE] " + line);
    }
  }
}

void initUartToSlave() {  
  uartToSlaveESP.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW); // Set RE/DE low to receive
}

std::tuple<float, float, float> readMasterSensors() {
  float temperature = NAN;
  float humidity = NAN;
  float pressure_hpa = NAN;

  // --- Read AHT20 (temperature + humidity) ---
  if (aht_ok) {
    sensors_event_t humEvent, tempEvent;
    aht20.getEvent(&humEvent, &tempEvent);

    if (!isnan(tempEvent.temperature)) {
      temperature = tempEvent.temperature;
    }
    if (!isnan(humEvent.relative_humidity)) {
      humidity = humEvent.relative_humidity;
    }
  } else {
    logger("AHT20 not initialized!", "WARN");
  }

  // --- Read BMP280 (pressure) ---
  if (bmp_ok) {
    pressure_hpa = bmp280.readPressure() / 100.0; // convert Pa to hPa
  } else {
    logger("BMP280 not initialized!", "WARN");
  }

  String line = String("[MASTER] TEMP=") + String(temperature) + ", PRESS=" + String(pressure_hpa) + ", HUMIDITY=" + String(humidity);
  logger(line);

  return std::make_tuple(temperature, humidity, pressure_hpa);
}

void writeSlaveCommand(const String &cmd) {
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(50);
  uartToSlaveESP.println(cmd);
  uartToSlaveESP.flush();
  delayMicroseconds(100);
  digitalWrite(RE_DE_PIN, LOW);
  logger("[MASTER] Send command: " + cmd);

  if(cmd == "RESET")
  {
      // wait for slave to reboot
      delay(30000);
  }
}

void readSlaveSensors() {
  readSlaveSensorCounter++;

  printPendingSlaveMessages();
  writeSlaveCommand("ALL");

  // wait for response
  delay(100); // ms

  if(!uartToSlaveESP.available()) {
    logger("[ESP8266] No response.", "WARN");
    writeSlaveCommand("RESET");
    return;
  }

  // read all available lines
  while (uartToSlaveESP.available()) {
    String line = uartToSlaveESP.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      logger("[SLAVE] " + line);
    }
    if (line.startsWith("TEMP=") && line.indexOf("PRESS=") != -1) {
      int tempStart = line.indexOf("TEMP=") + 5;
      int tempEnd = line.indexOf(",", tempStart);
      int pressStart = line.indexOf("PRESS=") + 6;

      String tempStr = line.substring(tempStart, tempEnd);
      String pressStr = line.substring(pressStart);

      float tempValue_slave = tempStr.toFloat();
      float pressValue_slave = pressStr.toFloat();

      auto [tempValue_master, humidityValue_master, pressValue_master] = readMasterSensors();

      mqttPublishWaterLevelAndSensors(tempValue_slave, pressValue_slave, tempValue_master, humidityValue_master, pressValue_master);
    }
    else if (line.indexOf("ERR") != -1) {
      writeSlaveCommand("RESET");
    }
  }
}

void initBmp280Aht20() {
  Wire.begin(BMP280_PIN_SDA, BMP280_PIN_SCL);

  if (!bmp280.begin()) {
    logger("Could not find a valid BMP280 sensor, check wiring!", "WARN");
    bmp_ok = false;
  } else {
    logger("BMP280 sensor initialized.");
    bmp_ok = true;
  }

  if (!aht20.begin()) {
    logger("Could not find a valid AHT20 sensor, check wiring!", "WARN");
    aht_ok = false;
  } else {
    logger("AHT20 sensor initialized.");
    aht_ok = true;
  }
}

// Subscribe helper: attempts to subscribe to config topic and sets mqtt_subscribed
void mqttSubscribeConfig() {
  if (mqttClient.subscribe(mqtt_esp32_config_topic)) {
    logger(String("Subscribed to topic: ") + String(mqtt_esp32_config_topic));
    mqtt_subscribed = true;
  } else {
    logger(String("Failed to subscribe to topic: ") + String(mqtt_esp32_config_topic), "WARN");
    mqtt_subscribed = false;
  }
}

void readMemory()
{
  logger("Reading stored statistics from NVS...");

  maxRecordedWaterLevel = memory.getFloat("maxWaterLev", 0.0);
  minRecordedWaterLevel = memory.getFloat("minWaterLev", 0.0);
  maxRecordedWaterLevelDate = memory.getString("maxWaterLevDate", "");
  minRecordedWaterLevelDate = memory.getString("minWaterLevDate", "");

  maxRecordedWaterIncreaseSpeed = memory.getFloat("maxWaterPosSpd", 0.0);
  maxRecordedWaterDecreaseSpeed = memory.getFloat("minWaterNegSpd", 0.0);
  maxRecordedWaterIncreaseSpeedDate = memory.getString("maxWaterPosSpdD", "");
  maxRecordedWaterDecreaseSpeedDate = memory.getString("minWaterNegSpdD", "");
}

void resetMemory() {
  logger("Resetting stored statistics in NVS...");
  memory.begin("myApp", false);
  memory.clear();
  delay(10);
  memory.putFloat("minWaterLev", 12.0); // start high
  memory.end();
  memory.begin("myApp", false);

  memory.putFloat("minWaterLev", 10.0); // start low
}

// IOTA 
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

void setup() {
  logger("Initializing...");
  espClient.setInsecure(); // disables certificate verification

  memory.begin("myApp", false); 
  readMemory();

  Serial.begin(115200);
  initUartToSlave();
  initTime();

  initBmp280Aht20();

  connectToWiFi();
  initOTA();

  mqttConnect();
}

void loop() {

  ArduinoOTA.handle();

  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  unsigned long now = millis();
  if (now - lastLoop >= READ_SENSOR_INTERVAL) {
    lastLoop = now;
    readSlaveSensors();
  }

  // publish water level stats periodically
  static unsigned long lastPublishMs = 0;
  unsigned long nowMs = millis();
  if (lastPublishMs == 0) lastPublishMs = nowMs;

  if (nowMs - lastPublishMs >= (unsigned long)WATER_STATS_PUBLISH_INTERVAL) {
    mqttPublishWaterLevelStats();
    lastPublishMs = nowMs;
  }
}
