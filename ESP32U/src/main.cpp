#include <DHT.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <time.h>

// --- DHT ---
#define DHTPIN 32
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// --- WiFi ---
const char* ssid = "4M";
const char* password = "Shxt-313";

// --- MQTT (HiveMQ) ---
const char* mqtt_server = "e492dd1e26cf46eb8faf8bf4c19894e2.s1.eu.hivemq.cloud"; // publiczny broker HiveMQ
const int mqtt_port = 8883;
const char* mqtt_topic = "esp32/dht22";
const char* mqtt_user = "esp_user";
const char* mqtt_password = "Rde11#aqaa";

// --- HTTP server ---
WebServer server(80);
// IPAddress local_IP(192, 168, 77, 249);  // IP dla ESP32
// IPAddress gateway(192, 168, 77, 1);     // Router (brama domyślna)
// IPAddress subnet(255, 255, 255, 0);     // Maska podsieci
// IPAddress primaryDNS(8, 8, 8, 8);       // DNS Google (opcjonalnie)
// IPAddress secondaryDNS(8, 8, 4, 4);     // DNS Google (opcjonalnie)

// String getHTMLPage(float temp) {
//   String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
//   html += "<meta http-equiv='refresh' content='2'>";  // auto-odświeżanie co 2 sekundy
//   html += "<title>Temperatura i Wilgotność</title></head><body style='font-family:sans-serif;text-align:center;'>";
//   html += "<h1>Odczyt z czujnika DHT22</h1>";
//   html += "<p style='font-size:24px;'>Temperatura: <b>" + String(temp) + " &deg;C</b></p>";
//   html += "<p>Odświeżanie co 2 sekundy</p>";
//   html += "</body></html>";
//   return html;
// }

// --- Funkcje NTP ---
void initTime() {
  configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
}

WiFiClientSecure espClient;
PubSubClient client(espClient);

String getFormattedTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "Brak czasu (NTP nie działa)";
  }
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

void scanWifi() {
  int n = WiFi.scanNetworks();
  Serial.println("Dostępne sieci WiFi:");
  for (int i = 0; i < n; ++i) {
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(WiFi.SSID(i));
    Serial.print(" (RSSI: ");
    Serial.print(WiFi.RSSI(i));
    Serial.print(" dBm) ");
    Serial.print((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Otwarta" : "Zabezpieczona");
    Serial.println();
  }
  if (n == 0) {
    Serial.println("Brak dostępnych sieci WiFi");
  }
  Serial.println();
}

void connectToWiFi() {
  Serial.print("Łączenie z siecią WiFi: ");
  Serial.println(ssid);
  // WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    // Serial.print("\n");

    scanWifi();
  }

  Serial.println("");
  Serial.println("Połączono z WiFi");
  Serial.print("Adres IP: ");
  Serial.println(WiFi.localIP());
}

// --- Połączenie z MQTT ---
void connectMQTT() {
  client.setServer(mqtt_server, mqtt_port);
  while (!client.connected()) {
    Serial.print("Łączenie z MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("połączono!");
    } else {
      Serial.print("błąd, rc=");
      Serial.print(client.state());
      Serial.println(", próba ponownie w 2s");
      delay(2000);
    }
  }
}

void mqttPublishSensorData() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  if (isnan(temp) || isnan(hum)) {
    Serial.println("Błąd odczytu DHT!");
    return;
  }

  long rssi = WiFi.RSSI();
  String datetime = getFormattedTime();

  // Tworzenie JSON
  String now = getFormattedTime();
  String date = now.substring(0, 10);   // "YYYY-MM-DD"
  String time = now.substring(11);      // "HH:MM:SS"
  String payload = "{\"temperature\":" + String(temp) +
                   ",\"humidity\":" + String(hum) +
                   ",\"date\":\"" + String(date) + "\"" +
                   ",\"time\":\"" + String(time) + "\"" +
                   ",\"rssi\":" + String(rssi) + "}";

  // Wysyłka do MQTT
  if (client.connected()) {
    client.publish(mqtt_topic, payload.c_str());
    Serial.println("Send to MQTT topic: " + String(mqtt_topic));
    Serial.println("Payload: " + payload);
  } else {
    Serial.println("MQTT niepołączone, próba ponownie...");
    connectMQTT(); // funkcja do ponownego połączenia
  }
}

void handleMeasurement() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>ESP32 DHT</title></head><body>";
  if (isnan(temp) || isnan(hum)) {
    html += "<h2>Błąd odczytu z DHT!</h2>";
    Serial.println("Błąd odczytu z DHT!");
  } else {
    String now = getFormattedTime();
    long rssi = WiFi.RSSI();
    
    html += "<p>Data i czas: " + now + "</p>";
    html += "<p>Siła sygnału WiFi (RSSI): " + String(rssi) + " dBm</p>";
    html += "<h2>Odczyty z czujnika DHT</h2>";
    html += "<p>Temperatura: " + String(temp) + " °C</p>";
    html += "<p>Wilgotność: " + String(hum) + " %</p>";
  }
  html += "</body></html>";

  server.send(200, "text/html", html);
  // Serial.println("Wysylanie pomiaru na Adres IP: ");
  // Serial.println(WiFi.localIP());

  // Publikowanie do MQTT w formacie JSON
  String now = getFormattedTime();
  long rssi = WiFi.RSSI();
  // Rozdziel datę i czas
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
  Serial.println("Serwer HTTP uruchomiony");
}

void setup() {
  espClient.setInsecure(); // wyłącza weryfikację certyfikatu

  Serial.begin(115200);
  initTime();
  dht.begin();
  connectToWiFi();
  connectMQTT();
  setupHttpServer();
}

unsigned long lastMsg = 0; // Move outside loop to retain value between iterations

void loop() {
  server.handleClient();
  // maintain MQTT connection
  client.loop();
  // reconnect MQTT if connection lost
  if (!client.connected()) {
    connectMQTT();
  }

  unsigned long now = millis();
  // Serial.println("Timestamp: " + String(now) + " diff: " + String(now - lastMsg));
  if (now - lastMsg > 10000) {  // co 5 sekund
    lastMsg = now;
    mqttPublishSensorData();
  }

  delay(2000);
}