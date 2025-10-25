#include <DHT.h>
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

// --- DHT ---
#define DHTPIN 32
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// --- WiFi ---
const char* ssid = "4M";
const char* password = "Shxt-313";
// const char* ssid = "Mnet_0";
// const char* password = "Vgt15rst";

// --- MQTT (HiveMQ) ---
const char* mqtt_server = "e492dd1e26cf46eb8faf8bf4c19894e2.s1.eu.hivemq.cloud"; // public HiveMQ broker
const int mqtt_port = 8883;
const char* mqtt_topic = "esp32/dht22";
const char* mqtt_waterlevel_topic = "esp32/studnia";
const char* mqtt_logs_topic = "esp32/studnia/logs";
const char* mqtt_user = "esp_user";
const char* mqtt_password = "Rde11#aqaa";

// --- HTTP server ---
WebServer server(80);

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
void connectMQTT() {
  mqttClient.setServer(mqtt_server, mqtt_port);
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected!");
    } else {
      Serial.print("error, rc=");
      Serial.print(mqttClient.state());
      Serial.println(", trying again in 2s");
      delay(2000);
    }
  }
}

void mqttPublishSensorData() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  if (isnan(temp) || isnan(hum)) {
    Serial.println("DHT read error!");
    return;
  }

  long rssi = WiFi.RSSI();
  String datetime = getFormattedTime();

  // Create JSON
  String now = getFormattedTime();
  String date = now.substring(0, 10);   // "YYYY-MM-DD"
  String time = now.substring(11);      // "HH:MM:SS"
  String payload = "{\"temperature\":" + String(temp) +
                   ",\"humidity\":" + String(hum) +
                   ",\"date\":\"" + String(date) + "\"" +
                   ",\"time\":\"" + String(time) + "\"" +
                   ",\"rssi\":" + String(rssi) + "}";

  // Send to MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_topic, payload.c_str());
    Serial.println("Send to MQTT topic: " + String(mqtt_topic));
    Serial.println("Payload: " + payload);
  } else {
    Serial.println("MQTT not connected, trying again...");
    connectMQTT(); // function to reconnect
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
  String now = getFormattedTime();
  String fullMessage = now + " | " + logMessage;
  String payload = "{\"log\":\"" + fullMessage + "\"}";

  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_logs_topic, payload.c_str());
    Serial.println("Sent log to MQTT topic: " + String(mqtt_logs_topic));
    Serial.println("Log Payload: " + payload);
  } else {
    Serial.println("MQTT not connected, trying again...");
    connectMQTT(); // function to reconnect
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
    connectMQTT(); // function to reconnect
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

void handleMeasurement() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>ESP32 DHT</title></head><body>";
  if (isnan(temp) || isnan(hum)) {
    html += "<h2>DHT read error!</h2>";
    Serial.println("DHT read error!");
  } else {
    String now = getFormattedTime();
    long rssi = WiFi.RSSI();
    
    html += "<p>Date and time: " + now + "</p>";
    html += "<p>WiFi signal strength (RSSI): " + String(rssi) + " dBm</p>";
    html += "<h2>DHT Sensor Readings</h2>";
    html += "<p>Temperature: " + String(temp) + " °C</p>";
    html += "<p>Humidity: " + String(hum) + " %</p>";
  }
  html += "</body></html>";

  server.send(200, "text/html", html);

  // Publish to MQTT in JSON format
  String now = getFormattedTime();
  long rssi = WiFi.RSSI();
  // Split date and time
  String date = now.substring(0, 10);   // "YYYY-MM-DD"
  String time = now.substring(11);      // "HH:MM:SS"
  String payload = "{\"temperature\":" + String(temp) +
                   ",\"humidity\":" + String(hum) +
                   ",\"date\":\"" + String(date) + "\"" +
                   ",\"time\":\"" + String(time) + "\"" +
                   ",\"rssi\":" + String(rssi) + "}";
  Serial.println("Send to MQTT topic: " + String(mqtt_topic));
  Serial.println("Payload: " + String(payload));
  mqttClient.publish(mqtt_topic, payload.c_str());
}

void setupHttpServer() {
  server.on("/", handleMeasurement);
  server.begin();
  Serial.println("HTTP server started");
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

  // --- Print nicely ---
  // Serial.print("Temperature: ");
  // Serial.print(temperature, 2);
  // Serial.println(" °C");

  // Serial.print("Humidity: ");
  // Serial.print(humidity, 2);
  // Serial.println(" %");

  // Serial.print("Pressure: ");
  // Serial.print(pressure_hpa, 2);
  // Serial.println(" hPa\n");

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
}

void readSlaveSensors() {
  printPendingSlaveMessages();
  writeSlaveCommand("ALL");

  // wait for response
  delay(100); // ms

  if(!uartToSlaveESP.available()) {
    Serial.println("[ESP8266] No response.");
    writeSlaveCommand("RESET");
    delay(5000);
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

      // Serial.print("Extracted TEMP: ");
      // Serial.print(tempValue_slave);
      // Serial.print(", PRESS: ");
      // Serial.println(pressValue_slave);

      auto [tempValue_master, humidityValue_master, pressValue_master] = readMasterSensors();

      mqttPublishWaterLevelAndSensors(tempValue_slave, pressValue_slave, tempValue_master, humidityValue_master, pressValue_master);
    }
    else if (line.indexOf("ERR") != -1) {
      writeSlaveCommand("RESET");
      delay(5000);
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

void setup() {
  Serial.println("Initializing...");
  espClient.setInsecure(); // disables certificate verification

  Serial.begin(115200);
  initUartToSlave();
  initTime();
  // dht.begin();

  initBmp280Aht20();

  connectToWiFi();
  connectMQTT();
  // setupHttpServer();
}

unsigned long lastMsg = 0; // Move outside loop to retain value between iterations

void loop() {

  readSlaveSensors();

  // std::pair<float,float> result = std::make_pair(NAN, NAN);
  // if (bmp_ok && aht_ok) {
  //   auto [temp, hum, press] = readMasterSensors();
  //     Serial.print("Temp=");
  //     Serial.print(temp);
  //     Serial.print("°C, Hum=");
  //     Serial.print(hum);
  //     Serial.print("%, Press=");
  //     Serial.print(press);
  //     Serial.println(" hPa");
  // }

  // if (!isnan(result.first)) {
  //   tempValue = result.first;
  //   pressValue = result.second / 100.0; // convert Pa to mbar
  // }

  delay(5000); // main loop delay
}
