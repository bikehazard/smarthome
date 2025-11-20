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
#include <Preferences.h>

const float CRITICAL_WATER_LEVEL          = 12.5; // m
const float RED_WATER_LEVEL_PERCENTAGE    = 90.0; // %
const float YELLOW_WATER_LEVEL_PERCENTAGE = 70.0; // %

enum State { GREEN_S, YELLOW_S, RED_S };
static State lastState = GREEN_S;

Preferences memory;

// --- WiFi ---
const char* ssid = "4M";
const char* password = "Shxt-313";

// --- LED diode ---
#define PIN_RED   18
#define PIN_GREEN 19
#define PIN_BLUE  21

// --- LCD setup ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define LCD_PIN_SDA 32   // SDA
#define LCD_PIN_SCL 33   // SCL

// --- MQTT (HiveMQ) ---
const char* mqtt_server = "e492dd1e26cf46eb8faf8bf4c19894e2.s1.eu.hivemq.cloud"; // public HiveMQ broker
const int mqtt_port = 8883;
const char* mqtt_waterlevel_topic = "esp32/studnia";
const char* mqtt_user = "esp_user";
const char* mqtt_password = "Rde11#aqaa";
bool mqtt_subscribed = false;

// ===== MQTT Function prototypes =====
void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttConnect();
void mqttReconnect();
void mqttSubscribeConfig();

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

void initDiode() {
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);

  ledcAttachPin(PIN_RED, 0);
  ledcAttachPin(PIN_GREEN, 1);
  ledcAttachPin(PIN_BLUE, 2);
}

void setDiodeColor(String color) {
  color.toLowerCase();

  if (color == "red") {
    ledcWrite(0, 255);  // R
    ledcWrite(1, 0);    // G
    ledcWrite(2, 0);    // B
  }
  else if (color == "green") {
    ledcWrite(0, 0);
    ledcWrite(1, 255);
    ledcWrite(2, 0);
  }
  else if (color == "blue") {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 255);
  }
  else if (color == "yellow") {
    ledcWrite(0, 255);
    ledcWrite(1, 255);
    ledcWrite(2, 0);
  }
  else if (color == "cyan") {
    ledcWrite(0, 0);
    ledcWrite(1, 255);
    ledcWrite(2, 255);
  }
  else if (color == "magenta") {
    ledcWrite(0, 255);
    ledcWrite(1, 0);
    ledcWrite(2, 255);
  }
  else if (color == "white") {
    ledcWrite(0, 255);
    ledcWrite(1, 255);
    ledcWrite(2, 255);
  }
  else if (color == "off" || color == "black") {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
  }
  else if (color == "orange") {
    ledcWrite(0, 255);  // R
    ledcWrite(1, 165);  // G
    ledcWrite(2, 0);    // B
  }
  else if (color == "purple") {
    ledcWrite(0, 128);
    ledcWrite(1, 0);
    ledcWrite(2, 128);
  }
  else if (color == "pink") {
    ledcWrite(0, 255);
    ledcWrite(1, 105);
    ledcWrite(2, 180);
  }
  else if (color == "lime") {
    ledcWrite(0, 191);
    ledcWrite(1, 255);
    ledcWrite(2, 0);
  }
  else if (color == "aqua") {
    ledcWrite(0, 0);
    ledcWrite(1, 255);
    ledcWrite(2, 255);
  }
  else {
    Serial.println("[WARN] Unknown color: " + color);
  }
}

void diodeNotifyWaterLevel(float percentageLevel) {
  const float HYST = 1.0; // hysteresis margin in percent

  logger("diodeNotifyWaterLevel: level=" + String(percentageLevel) + "%, lastState=" +
         (lastState == RED_S ? "RED" : (lastState == YELLOW_S ? "YELLOW" : "GREEN")), "DEBUG");

  State nextState = lastState;

  if (percentageLevel >= RED_WATER_LEVEL_PERCENTAGE) {
    logger("Level >= RED threshold (" + String(RED_WATER_LEVEL_PERCENTAGE) + "%)", "DEBUG");
    nextState = RED_S;
  } else if (percentageLevel >= YELLOW_WATER_LEVEL_PERCENTAGE) {
    logger("Level >= YELLOW threshold (" + String(YELLOW_WATER_LEVEL_PERCENTAGE) + "%)", "DEBUG");
    // If we were RED, stay RED until it falls below RED - HYST
    if (lastState == RED_S && percentageLevel > (RED_WATER_LEVEL_PERCENTAGE - HYST)) {
      logger("Hysteresis active: staying RED (level " + String(percentageLevel) +
             " > RED-HYST " + String(RED_WATER_LEVEL_PERCENTAGE - HYST) + ")", "DEBUG");
      nextState = RED_S;
    } else {
      nextState = YELLOW_S;
    }
  } else {
    // Below YELLOW threshold
    logger("Level < YELLOW threshold", "DEBUG");
    // If we were YELLOW, stay YELLOW until it falls below YELLOW - HYST
    if (lastState == YELLOW_S && percentageLevel > (YELLOW_WATER_LEVEL_PERCENTAGE - HYST)) {
      logger("Hysteresis active: staying YELLOW (level " + String(percentageLevel) +
             " > YELLOW-HYST " + String(YELLOW_WATER_LEVEL_PERCENTAGE - HYST) + ")", "DEBUG");
      nextState = YELLOW_S;
    } else {
      nextState = GREEN_S;
    }
  }

  if (nextState != lastState) {
    logger("State change: " +
           (lastState == RED_S ? String("RED") : (lastState == YELLOW_S ? String("YELLOW") : String("GREEN"))) +
           " -> " +
           (nextState == RED_S ? String("RED") : (nextState == YELLOW_S ? String("YELLOW") : String("GREEN"))),
           "INFO");

    switch (nextState) {
      case RED_S:    setDiodeColor("red");    break;
      case YELLOW_S: setDiodeColor("yellow"); break;
      case GREEN_S:  setDiodeColor("green");  break;
    }
    lastState = nextState;
  } else {
    logger("No state change: remains " +
           (nextState == RED_S ? String("RED") : (nextState == YELLOW_S ? String("YELLOW") : String("GREEN"))),
           "DEBUG");
  }
}

void testAllColors() {
  setDiodeColor("red");
  delay(3000);

  setDiodeColor("green");
  delay(3000);

  setDiodeColor("blue");
  delay(3000);

  setDiodeColor("yellow");
  delay(3000);

  setDiodeColor("cyan");
  delay(3000);

  setDiodeColor("magenta");
  delay(3000);

  setDiodeColor("white");
  delay(3000);

  setDiodeColor("orange");
  delay(3000);

  setDiodeColor("purple");
  delay(3000);

  setDiodeColor("pink");
  delay(3000);

  setDiodeColor("lime");
  delay(3000);

  setDiodeColor("aqua");
  delay(3000);

  setDiodeColor("off");  // wyłączenie diody
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

float getFloatFromKey(const String &msg, const String &key) {
  int idx = msg.indexOf(key + "\":");
  if(idx == -1) return NAN;
  int start = idx + key.length() + 2;
  int end = msg.indexOf(',', start);
  if(end == -1) end = msg.length();
  return msg.substring(start, end).toFloat();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = String((const char*)payload, length);
  // logger("[MQTT] Callback invoked. Topic: " + String(topic) + " ,message: " + message);

  mqttData.waterLevel     = getFloatFromKey(message, "waterLevel");
  mqttData.tempSlave      = getFloatFromKey(message, "tempSlave");
  mqttData.pressSlave     = getFloatFromKey(message, "pressSlave");
  mqttData.tempMaster     = getFloatFromKey(message, "tempMaster");
  mqttData.humidityMaster = getFloatFromKey(message, "humidityMaster");
  mqttData.pressMaster    = getFloatFromKey(message, "pressMaster");
  mqttData.rssi           = getFloatFromKey(message, "rssi");

  // logger("waterLevel: " + String(mqttData.waterLevel));
  // logger("tempSlave: " + String(mqttData.tempSlave));
  // logger("pressSlave: " + String(mqttData.pressSlave));
  // logger("tempMaster: " + String(mqttData.tempMaster));
  // logger("humidityMaster: " + String(mqttData.humidityMaster));
  // logger("pressMaster: " + String(mqttData.pressMaster));
  // logger("rssi: " + String(mqttData.rssi));

  float percentageLevel = mqttData.waterLevel / CRITICAL_WATER_LEVEL * 100.0;
  lcd_print("Studnia: " + String(percentageLevel, 1) + "%", "");
  diodeNotifyWaterLevel(percentageLevel);
}

void mqttConnect() {
  // Configure client and hand off connection logic to mqttReconnect()
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttReconnect();
}

// Subscribe helper: attempts to subscribe to config topic and sets mqtt_subscribed
void mqttSubscribeConfig() {
  if (mqttClient.subscribe(mqtt_waterlevel_topic, 0)) {
    logger(String("Subscribed to topic: ") + String(mqtt_waterlevel_topic));
    mqtt_subscribed = true;
  } else {
    logger(String("Failed to subscribe to topic: ") + String(mqtt_waterlevel_topic), "WARN");
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


void setup() {
  lcd_init();
  lcd_print("ESP32", "Initializing...");

  memory.begin("myApp", false); 

  // float tempValue = 23.45;
  // preferences.putFloat("temp", tempValue);
  // preferences.putString("name", "ESP32");

  logger("Initializing...");
  espClient.setInsecure(); // disables certificate verification

  initDiode();
  testAllColors();

  Serial.begin(115200);
  connectToWiFi();
  initTimeAndWait();

  mqttConnect();

  // odczyt pamieci
  // memory.clear(); // odkomentuj, aby wyczyscic pamiec
  // float tempRead = memory.getFloat("temp", 0.0);
  // Serial.println("Odczyt temp: " + String(tempRead));
  // String nameRead = memory.getString("name", "unknown");
  // Serial.println("Odczyt name: " + nameRead);
}

void loop() {

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

  // lcd.scrollDisplayLeft();
  // delay(300);
  delay(50);
}