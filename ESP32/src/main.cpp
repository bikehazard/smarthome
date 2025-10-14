#include <DHT.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <time.h>
#include <Arduino.h>
#include <Wire.h>

// ======================= UART (HardwareSerial) =====================
#define RX_PIN 16
#define TX_PIN 17
#define RE_DE_PIN 19
#define UART_BAUD 9600
HardwareSerial uartToSlaveESP(1);

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
const char* mqtt_user = "esp_user";
const char* mqtt_password = "Rde11#aqaa";

// --- HTTP server ---
WebServer server(80);

// --- NTP Functions ---
void initTime() {
  configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
}

WiFiClientSecure espClient;
PubSubClient client(espClient);

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
  client.setServer(mqtt_server, mqtt_port);
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected!");
    } else {
      Serial.print("error, rc=");
      Serial.print(client.state());
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
  if (client.connected()) {
    client.publish(mqtt_topic, payload.c_str());
    Serial.println("Send to MQTT topic: " + String(mqtt_topic));
    Serial.println("Payload: " + payload);
  } else {
    Serial.println("MQTT not connected, trying again...");
    connectMQTT(); // function to reconnect
  }
}

float calculateWaterLevel(float pressure) {
  // Assuming pressure is in mbar and the density of water is 1000 kg/m^3
  // Water level (in meters) = Pressure (in mbar) / (density of water * g) * 100
  // where g is the acceleration due to gravity (approximately 9.81 m/s^2)
  float waterLevel = pressure / (1000 * 9.81); // meters
  Serial.print("Calculated Water Level: ");
  Serial.print(waterLevel);
  Serial.println(" m");
  return waterLevel;
}

void mqttPublishWaterLevel(float pressValue) {

  float waterLevel = calculateWaterLevel(pressValue);

  long rssi = WiFi.RSSI();
  String datetime = getFormattedTime();

  // create JSON
  String now = getFormattedTime();
  String date = now.substring(0, 10);   // "YYYY-MM-DD"
  String time = now.substring(11);      // "HH:MM:SS"
  String payload = "{\"waterLevel\":" + String(waterLevel) +
                   ",\"date\":\"" + String(date) + "\"" +
                   ",\"time\":\"" + String(time) + "\"" +
                   ",\"rssi\":" + String(rssi) + "}";

  // send to MQTT
  if (client.connected()) {
    client.publish(mqtt_waterlevel_topic, payload.c_str());
    Serial.println("Send to MQTT topic: " + String(mqtt_waterlevel_topic));
    Serial.println("Payload: " + payload);
  } else {
    Serial.println("MQTT not connected, trying again...");
    connectMQTT(); // function to reconnect
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
  client.publish(mqtt_topic, payload.c_str());
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

void readMS5803FromESP8266(const String &cmd) {
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(50);
  uartToSlaveESP.println(cmd);
  uartToSlaveESP.flush();
  delayMicroseconds(100);
  digitalWrite(RE_DE_PIN, LOW);
  Serial.println("[ESP32] Send command: " + cmd);

  // wait for response
  delay(100); // ms

  if(!uartToSlaveESP.available()) {
    Serial.println("[ESP8266] No response.");
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

      float tempValue = tempStr.toFloat();
      float pressValue = pressStr.toFloat();

      Serial.print("Extracted TEMP: ");
      Serial.print(tempValue);
      Serial.print(", PRESS: ");
      Serial.println(pressValue);

      mqttPublishWaterLevel(pressValue);
    }
  }
}

void setup() {
  Serial.println("Initializing...");
  espClient.setInsecure(); // disables certificate verification

  Serial.begin(115200);
  initUartToSlave();
  initTime();
  // dht.begin();

  connectToWiFi();
  connectMQTT();
  // setupHttpServer();
}

unsigned long lastMsg = 0; // Move outside loop to retain value between iterations

void loop() {
  // server.handleClient();
  // // maintain MQTT connection
  // client.loop();
  // // reconnect MQTT if connection lost
  // if (!client.connected()) {
  //   connectMQTT();
  // }

  // unsigned long now = millis();
  // // Serial.println("Timestamp: " + String(now) + " diff: " + String(now - lastMsg));
  // if (now - lastMsg > 10000) {  // every 5 seconds
  //   lastMsg = now;
  //   mqttPublishSensorData();
  // }

  readMS5803FromESP8266("ALL");

  delay(2000);
}
