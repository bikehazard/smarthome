#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <time.h>
#include <Arduino.h>
#include <utility>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

// thresholds are percentages (0..100)
const float YELLOW_WATER_LEVEL_PERCENTAGE = 0.02; // %
const float RED_WATER_LEVEL_PERCENTAGE    = 0.05; // %
const float HYST = 0.01; // hysteresis margin in percent
const unsigned long NOTIFY_TIMEOUT_MS = 60ul * 1000ul; // 60 seconds

enum State { GREEN_S, YELLOW_S, RED_S };
// track last state separately for the two diodes
static State lastStateStudnia = GREEN_S;
static State lastStateZbiornik = GREEN_S;

struct lcdDataStruct {
  String line1;
  String line2;
};

lcdDataStruct lcdData;

// --- Diode timeout tracking ---
static unsigned long lastNotifyStudnia = 0;
static unsigned long lastNotifyZbiornik = 0;
static bool studniaTimedOut = false;
static bool zbiornikTimedOut = false;

// --- WiFi ---
const char* ssid = "4M";
const char* password = "Shxt-313";
const char* host = "esp32-updater-lcd-monitoring";

// --- LED diode ---
#define STUDNIA_LED_PIN_RED   18
#define STUDNIA_LED_PIN_GREEN 19
#define STUDNIA_LED_PIN_BLUE  21

#define ZBIORNIK_LED_PIN_RED   14
#define ZBIORNIK_LED_PIN_GREEN 12
#define ZBIORNIK_LED_PIN_BLUE  13

// Which diode to target when setting color
enum Diode { STUDNIA = 0, ZBIORNIK = 1 };

// --- LCD setup ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define LCD_PIN_SDA 32   // SDA
#define LCD_PIN_SCL 33   // SCL

// --- MQTT (HiveMQ) ---
const char* mqtt_server = "e492dd1e26cf46eb8faf8bf4c19894e2.s1.eu.hivemq.cloud"; // public HiveMQ broker
const int mqtt_port = 8883;
const char* mqtt_waterlevel_studnia_topic = "esp32/studnia";
const char* mqtt_waterlevel_zbiornik_topic = "esp32/zbiornik";
const char* mqtt_user = "esp_user";
const char* mqtt_password = "Rde11#aqaa";
bool mqtt_subscribed = false;

// ===== MQTT Function prototypes =====
void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttConnect();
void mqttReconnect();
void mqttSubscribeConfig();

// ===== Other Function prototypes =====
void logger(const String &msg, const String &severity);
String getFormattedTime();
void initTimeAndWait();
void connectToWiFi();

// LCD
void lcd_init();
void lcd_print(const String &line1, const String &line2);

// Diodes / LEDs
void initDiodes();
void setDiodeColor(const String &colorIn, Diode diode);
void diodeNotifyWaterLevel(float percentageLevel, Diode diode);
void testAllColors();

// MQTT handlers
float getFloatFromKey(const String &msg, const String &key);
void handleCallbackStudnia(byte* payload, unsigned int length);
void handleCallbackZbiornik(byte* payload, unsigned int length);

// logger
void logger(const String &msg, const String &severity = "INFO") {
  String formatted = String("[") + severity + "] " + msg;
  Serial.println(formatted);
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

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

void connectToWiFi() {
  // WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  logger(String("Connected to WiFi: ") + String(ssid));
  logger(String("IP Address: ") + WiFi.localIP().toString());
}

// --- LCD ---

void lcd_init() {
  logger("Initializing LCD...");
  Wire.begin(LCD_PIN_SDA, LCD_PIN_SCL);
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void lcd_print(const String &line1, const String &line2) {
  lcd.clear();

  // cut to 16 chars
  String l1 = line1;
  if (l1.length() > 16) l1 = l1.substring(0, 16);

  String l2 = line2;
  if (l2.length() > 16) l2 = l2.substring(0, 16);

  lcd.setCursor(0, 0);
  lcd.print(l1);
  lcd.setCursor(0, 1);
  lcd.print(l2);
}

// --- LED Diode ---

void initDiodes() {
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);

  ledcSetup(3, 5000, 8);
  ledcSetup(4, 5000, 8);
  ledcSetup(5, 5000, 8);

  ledcAttachPin(STUDNIA_LED_PIN_RED, 0);
  ledcAttachPin(STUDNIA_LED_PIN_GREEN, 1);
  ledcAttachPin(STUDNIA_LED_PIN_BLUE, 2);

  ledcAttachPin(ZBIORNIK_LED_PIN_RED, 3);
  ledcAttachPin(ZBIORNIK_LED_PIN_GREEN, 4);
  ledcAttachPin(ZBIORNIK_LED_PIN_BLUE, 5);

  setDiodeColor("blue", ZBIORNIK);
  setDiodeColor("blue", STUDNIA);
}

// Set a color on one of the two RGB diodes.
// diodeIndex: 
// 0 = Studnia (channels 0..2), 
// 1 = Zbiornik (channels 3..5)
void setDiodeColor(const String &colorIn, Diode diode = STUDNIA) {
  String color = colorIn;
  color.toLowerCase();
  int base = (diode == ZBIORNIK) ? 3 : 0;

  auto writeRGB = [&](int r, int g, int b){
    ledcWrite(base + 0, r);
    ledcWrite(base + 1, g);
    ledcWrite(base + 2, b);
  };

  if (color == "red") {
    writeRGB(255, 0, 0);
  }
  else if (color == "green") {
    writeRGB(0, 255, 0);
  }
  else if (color == "blue") {
    writeRGB(0, 0, 255);
  }
  else if (color == "yellow") {
    writeRGB(255, 255, 0);
  }
  else if (color == "cyan") {
    writeRGB(0, 255, 255);
  }
  else if (color == "magenta") {
    writeRGB(255, 0, 255);
  }
  else if (color == "white") {
    writeRGB(255, 255, 255);
  }
  else if (color == "off" || color == "black") {
    writeRGB(0, 0, 0);
  }
  else if (color == "orange") {
    writeRGB(255, 165, 0);
  }
  else if (color == "purple") {
    writeRGB(128, 0, 128);
  }
  else if (color == "pink") {
    writeRGB(255, 105, 180);
  }
  else if (color == "lime") {
    writeRGB(191, 255, 0);
  }
  else if (color == "aqua") {
    writeRGB(0, 255, 255);
  }
  else {
    Serial.println(String("[WARN] Unknown color: ") + color);
  }
}

void diodeNotifyWaterLevel(float percentageLevel, Diode diode) {
  // update last-notify timestamp for this diode so the timeout watchdog
  // knows this diode was recently handled
  if (diode == STUDNIA) {
    lastNotifyStudnia = millis();
    studniaTimedOut = false;
  } else {
    lastNotifyZbiornik = millis();
    zbiornikTimedOut = false;
  }

  String which = (diode == STUDNIA) ? "Studnia" : "Zbiornik";
  logger("diodeNotifyWaterLevel: " + which + "=" + String(percentageLevel) + "%", "DEBUG");

  // calculate next state for this diode only (simplified with hysteresis)
  auto calcState = [&](float percentage, State last)->State {
    
    // up thresholds (apply hysteresis)
    const float red_up = RED_WATER_LEVEL_PERCENTAGE + HYST;
    const float yellow_up = YELLOW_WATER_LEVEL_PERCENTAGE + HYST;

    // down thresholds (apply hysteresis)
    const float red_down = RED_WATER_LEVEL_PERCENTAGE - HYST;
    const float yellow_down = YELLOW_WATER_LEVEL_PERCENTAGE - HYST;

    // moving up: cross absolute thresholds
    if (percentage >= red_up) return RED_S;
    if (percentage >= yellow_up) return YELLOW_S;

    // moving down: stay in higher state until below the lowered threshold
    if (last == RED_S && percentage >= red_down) return RED_S;
    if (last == YELLOW_S && percentage >= yellow_down) return YELLOW_S;

    // otherwise we're green
    return GREEN_S;
  };

  if (diode == STUDNIA) {
    State next = calcState(percentageLevel, lastStateStudnia);
    logger("Studnia set to " +
           (next == RED_S ? String("RED") : (next == YELLOW_S ? String("YELLOW") : String("GREEN"))),
           "INFO");
    switch (next) {
      case RED_S:    setDiodeColor("red", STUDNIA);    break;
      case YELLOW_S: setDiodeColor("yellow", STUDNIA); break;
      case GREEN_S:  setDiodeColor("green", STUDNIA);  break;
    }
    lastStateStudnia = next;
  } else {
    State next = calcState(percentageLevel, lastStateZbiornik);
    logger("Zbiornik set to " +
           (next == RED_S ? String("RED") : (next == YELLOW_S ? String("YELLOW") : String("GREEN"))),
           "INFO");
    switch (next) {
      case RED_S:    setDiodeColor("red", ZBIORNIK);    break;
      case YELLOW_S: setDiodeColor("orange", ZBIORNIK); break;
      case GREEN_S:  setDiodeColor("green", ZBIORNIK);  break;
    }
    lastStateZbiornik = next;
  }
}

void testAllColors() {
  String colors[] = {"red","green","blue","yellow","cyan","magenta","white","orange","purple","pink","lime","aqua"};
  // String colors[] = {"red","green","yellow"};
  for (const String &c : colors) {
  // set both diodes for the visual test
  setDiodeColor(c, STUDNIA);
  setDiodeColor(c, ZBIORNIK);
    delay(3000);
  }
  setDiodeColor("off", STUDNIA);
  setDiodeColor("off", ZBIORNIK);  // wyłączenie diody
}

// --- MQTT Connection ---

struct MqttData {
  float waterLevel;
  float waterLevelPercentage;
  int rssi;
};

float getFloatFromKey(const String &msg, const String &key) {
  int idx = msg.indexOf(key + "\":");
  if(idx == -1) return NAN;
  int start = idx + key.length() + 2;
  int end = msg.indexOf(',', start);
  if(end == -1) end = msg.length();
  return msg.substring(start, end).toFloat();
}

void handleCallbackStudnia(byte* payload, unsigned int length) {
  String message = String((const char*)payload, length);
  logger("[MQTT] Studnia topic message received.");

  float waterLevel           = getFloatFromKey(message, "waterLev");
  float waterLevelPercentage = getFloatFromKey(message, "waterLevPerc");

  lcdData.line1 = "Studnia:  " + String(waterLevelPercentage, 1) + "%";
  lcd_print(lcdData.line1, lcdData.line2);
  diodeNotifyWaterLevel(waterLevelPercentage, STUDNIA);
}

void handleCallbackZbiornik(byte* payload, unsigned int length) {
  String message = String((const char*)payload, length);
  logger("[MQTT] Zbiornik topic message received.");

  float waterLevel           = getFloatFromKey(message, "waterLev");
  float waterLevelPercentage = getFloatFromKey(message, "waterLevPerc");

  lcdData.line2 = "Zbiornik: " + String(waterLevelPercentage, 1) + "%";
  lcd_print(lcdData.line1, lcdData.line2);
  diodeNotifyWaterLevel(waterLevelPercentage, ZBIORNIK);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = String((const char*)payload, length);
  // logger("[MQTT] Callback invoked. Topic: " + String(topic) + " ,message: " + message);

  if (String(topic) == mqtt_waterlevel_studnia_topic) {
    handleCallbackStudnia(payload, length);
  } 
  else if (String(topic) == mqtt_waterlevel_zbiornik_topic) {
    handleCallbackZbiornik(payload, length);
  } 
  else {
    logger(String("Unknown topic: ") + String(topic) + " ,message: " + message, "WARN");
    return;
  } 
}

void mqttConnect() {
  // Configure client and hand off connection logic to mqttReconnect()
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttReconnect();
}

// Subscribe helper: attempts to subscribe to config topic and sets mqtt_subscribed
void mqttSubscribeConfig() {
  if (mqttClient.subscribe(mqtt_waterlevel_studnia_topic, 0)) {
    logger(String("Subscribed to topic: ") + String(mqtt_waterlevel_studnia_topic));
    mqtt_subscribed = true;
  } else {
    logger(String("Failed to subscribe to topic: ") + String(mqtt_waterlevel_studnia_topic), "WARN");
    mqtt_subscribed = false;
  }

  if (mqttClient.subscribe(mqtt_waterlevel_zbiornik_topic, 0)) {
    logger(String("Subscribed to topic: ") + String(mqtt_waterlevel_zbiornik_topic));
    mqtt_subscribed = true;
  } else {
    logger(String("Failed to subscribe to topic: ") + String(mqtt_waterlevel_zbiornik_topic), "WARN");
    mqtt_subscribed = false;
  }
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    logger("Connecting to MQTT...");
    if (mqttClient.connect("ESP32Client_LCD", mqtt_user, mqtt_password)) {
      logger("Connected to MQTT broker.");
      mqttSubscribeConfig();
    } else {
      logger(String("Failed, rc=") + String(mqttClient.state()), "WARN");
      delay(2000);
    }
  }
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
      Serial.println("Start initialization " + type);
    })
    .onEnd([]() {
      Serial.println("\nInit done!");
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
  lcd_init();
  lcd_print("ESP32", "Initializing...");

  initDiodes();

  logger("Initializing...");
  espClient.setInsecure(); // disables certificate verification

  Serial.begin(115200);
  connectToWiFi();
  initTimeAndWait();

  initOTA();
  // initialize notify timestamps to now to avoid immediate timeout
  lastNotifyStudnia = millis();
  lastNotifyZbiornik = millis();

  // testAllColors();

  mqttConnect();

  lcdData.line1 = "Studnia:  N/A";
  lcdData.line2 = "Zbiornik: N/A";
  lcd_print(lcdData.line1, lcdData.line2);
}

void statusWatchdog() {
  // watchdog: if no diodeNotifyWaterLevel was called for a diode for more than
  // NOTIFY_TIMEOUT_MS, set that diode to blue to indicate stale data.
  unsigned long now = millis();
  if (!studniaTimedOut && (now - lastNotifyStudnia > NOTIFY_TIMEOUT_MS)) {
    logger("Studnia has not been updated for >60s, forcing BLUE color", "WARN");
    setDiodeColor("blue", STUDNIA);
    studniaTimedOut = true;
    lcdData.line1 = lcdData.line1 + "?";
    lcd_print(lcdData.line1, lcdData.line2);
  }
  if (!zbiornikTimedOut && (now - lastNotifyZbiornik > NOTIFY_TIMEOUT_MS)) {
    logger("Zbiornik has not been updated for >60s, forcing BLUE color", "WARN");
    setDiodeColor("blue", ZBIORNIK);
    zbiornikTimedOut = true;
    lcdData.line2 = lcdData.line2 + "?";
    lcd_print(lcdData.line1, lcdData.line2);
  }
}

void loop() {

  ArduinoOTA.handle();

  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  statusWatchdog();

  if(WiFi.status() != WL_CONNECTED) {
    logger("WiFi disconnected, reconnecting...");
    connectToWiFi();
  }

  if (mqttClient.state() != 0) {
    logger("MQTT internal state changed: " + String(mqttClient.state()));
  }

  delay(100);
}