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

unsigned long lastLoop = 0;
const unsigned long mainLoopInterval = 5000; // 5 seconds

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

// --- MQTT (HiveMQ) ---
const char* mqtt_server = "e492dd1e26cf46eb8faf8bf4c19894e2.s1.eu.hivemq.cloud"; // public HiveMQ broker
const int mqtt_port = 8883;
const char* mqtt_waterlevel_topic = "esp32/studnia";
const char* mqtt_logs_topic = "esp32/studnia/logs";
const char* mqtt_esp32_config_topic = "esp32/studnia/config";  // where broker sends config
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

      // Acknowledge the change back to logs topic (always send ack regardless of forward_logs state)
      // String ack = String("{\"DEBUG\":") + val + "}";
      // if (mqttClient.connected()) {
      //   mqttClient.publish(mqtt_logs_topic, ack.c_str());
      //   logger("Published DEBUG ack to MQTT: " + ack);
      // } else {
      //   logger("Cannot publish DEBUG ack, MQTT not connected", "WARN");
      // }
    } 
    else if (msg == "RESET") {
      logger("Config: RESET received -> rebooting ESP32...", "WARN");
      delay(2000);
      ESP.restart();
      delay(2000);
    } else {
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
    if (mqttClient.connect("ESP32Client", mqtt_user, mqtt_password)) {
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

void mqttPublishWaterLevelAndSensors(float tempValue_slave, float pressValue_slave, float tempValue_master, float humidityValue_master, float pressValue_master) {
  
  float waterLevel = calculateWaterLevel(pressValue_slave, pressValue_master);

  long rssi = WiFi.RSSI();
  String datetime = getFormattedTime();

  // create JSON
  String now = getFormattedTime();
  String date = now.substring(0, 10);   // "YYYY-MM-DD"
  String time = now.substring(11);      // "HH:MM:SS"
  String payload = "{\"waterLevel\":" + String(waterLevel) + "\"" +
                   ",\"tempSlave\":" + String(tempValue_slave) + "\"" +
                   ",\"pressSlave\":" + String(pressValue_slave) + "\"" +
                   ",\"tempMaster\":" + String(tempValue_master) + "\"" +
                   ",\"humidityMaster\":" + String(humidityValue_master) + "\"" +
                   ",\"pressMaster\":" + String(pressValue_master) + "\"" +
                   ",\"date\":\"" + String(date) + "\"" +
                   ",\"time\":\"" + String(time) + "\"" +
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

void setup() {
  logger("Initializing...");
  espClient.setInsecure(); // disables certificate verification

  Serial.begin(115200);
  initUartToSlave();
  initTime();

  initBmp280Aht20();

  connectToWiFi();
  mqttConnect();
}

void loop() {

  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  unsigned long now = millis();
  if (now - lastLoop >= mainLoopInterval) {
    lastLoop = now;

    readSlaveSensors();
  }
}
