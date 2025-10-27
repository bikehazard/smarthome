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
const char* ssid = "4M";
const char* password = "Shxt-313";
// const char* ssid = "Mnet_0";
// const char* password = "Vgt15rst";

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
bool mqtt_enable_slave_logs = true;

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

void scanWifi() {
  int n = WiFi.scanNetworks();
  Serial.println("Available WiFi networks:");
  for (int i = 0; i < n; ++i) {
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(WiFi.SSID(i));
    Serial.print(" (RSSI: ");
    Serial.print(WiFi.RSSI(i));
    Serial.print(" dBm) ");
    Serial.print((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Secured");
    Serial.println();
  }
  if (n == 0) {
    Serial.println("No available WiFi networks");
  }
  Serial.println();
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  // WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    scanWifi();
  }

  Serial.println("");
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// --- MQTT Connection ---

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.println("[MQTT] Callback invoked");
  Serial.printf("Message arrived [%s]: ", topic);
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // --- Process the config string here ---
  if (String(topic) == mqtt_esp32_config_topic) {
    String msg = message;
    msg.trim();
    msg.toUpperCase();

    if (msg.startsWith("{")) {
      Serial.println("Parsing JSON config...");
      // TODO: parse JSON configuration if needed
    } else if (msg.startsWith("DEBUG=")) {
      String val = msg.substring(msg.indexOf('=') + 1);
      val.trim();
      if (val == "1") {
        mqtt_enable_master_logs = true;
        Serial.println("Config: DEBUG=1 -> enabling log forwarding to MQTT");
      } else if (val == "0") {
        mqtt_enable_master_logs = false;
        Serial.println("Config: DEBUG=0 -> disabling log forwarding to MQTT");
      } else {
        Serial.printf("Unknown DEBUG value: %s\n", val.c_str());
      }

      // Acknowledge the change back to logs topic (always send ack regardless of forward_logs state)
      String ack = String("{\"DEBUG\":") + val + "}";
      if (mqttClient.connected()) {
        mqttClient.publish(mqtt_logs_topic, ack.c_str());
        Serial.println("Published DEBUG ack to MQTT: " + ack);
      } else {
        Serial.println("Cannot publish DEBUG ack, MQTT not connected");
      }
    } else if (msg == "RESET") {
      Serial.println("Config: RESET received -> rebooting ESP32...");
      // acknowledge reset request
      String ack = "{\"RESET\":\"OK\"}";
      if (mqttClient.connected()) {
        mqttClient.publish(mqtt_logs_topic, ack.c_str());
        Serial.println("Published RESET ack to MQTT: " + ack);
      } else {
        Serial.println("Cannot publish RESET ack, MQTT not connected");
      }
      delay(200);
      ESP.restart();
      delay(1000);
    } else {
      Serial.printf("Received config: %s\n", message.c_str());
      // For example, save to variable or preferences
    }
  }
}

void mqttConnect() {
  // Configure client and hand off connection logic to mqttReconnect()
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttReconnect();
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // ensure subscription is attempted after a successful connect
      mqttSubscribeConfig();
    } else {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
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

  Serial.print("Calculated Water Level: ");
  Serial.print(water_level_m);
  Serial.println(" m");
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
    Serial.println("Send to MQTT topic: " + String(mqtt_waterlevel_topic));
    Serial.println("Payload: " + payload);
  } else {
    Serial.println("MQTT not connected, trying again...");
    mqttConnect(); // function to reconnect
  }
}

void printPendingSlaveMessages() {
  while (uartToSlaveESP.available()) {
    String line = uartToSlaveESP.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.println("[ESP8266][PRE] " + line);
      publishLogToMQTT("[ESP8266] " + line);
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
    Serial.println("AHT20 not initialized!");
  }

  // --- Read BMP280 (pressure) ---
  if (bmp_ok) {
    pressure_hpa = bmp280.readPressure() / 100.0; // convert Pa to hPa
  } else {
    Serial.println("BMP280 not initialized!");
  }

  Serial.print("[ESP32] TEMP=");
  Serial.print(temperature);
  Serial.print(", PRESS=");
  Serial.print(pressure_hpa);
  Serial.print(", HUMIDITY=");
  Serial.print(humidity);

  return std::make_tuple(temperature, humidity, pressure_hpa);
}

void writeSlaveCommand(const String &cmd) {
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(50);
  uartToSlaveESP.println(cmd);
  uartToSlaveESP.flush();
  delayMicroseconds(100);
  digitalWrite(RE_DE_PIN, LOW);
  Serial.println("[ESP32] Send command: " + cmd);

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
    Serial.println("[ESP8266] No response.");
    writeSlaveCommand("RESET");
    return;
  }

  // read all available lines
  while (uartToSlaveESP.available()) {
    String line = uartToSlaveESP.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.println("[ESP8266] " + line);
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
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    bmp_ok = false;
  } else {
    Serial.println("BMP280 sensor initialized.");
    bmp_ok = true;
  }

  if (!aht20.begin()) {
    Serial.println("Could not find a valid AHT20 sensor, check wiring!");
    aht_ok = false;
  } else {
    Serial.println("AHT20 sensor initialized.");
    aht_ok = true;
  }
}

// Subscribe helper: attempts to subscribe to config topic and sets mqtt_subscribed
void mqttSubscribeConfig() {
  if (mqttClient.subscribe(mqtt_esp32_config_topic)) {
    Serial.printf("Subscribed to topic: %s\n", mqtt_esp32_config_topic);
    mqtt_subscribed = true;
  } else {
    Serial.printf("Failed to subscribe to topic: %s\n", mqtt_esp32_config_topic);
    mqtt_subscribed = false;
  }
}

void setup() {
  Serial.println("Initializing...");
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

    // readSlaveSensors();
    // publish data etc.
  }
}
