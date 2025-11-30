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

Preferences memory;

// --- WiFi ---
const char* ssid = "4M";
const char* password = "Shxt-313";

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
const char* mqtt_waterlevel_topic = "esp32/studnia";
const char* mqtt_user = "esp_user";
const char* mqtt_password = "Rde11#aqaa";
bool mqtt_subscribed = false;

// ===== MQTT Function prototypes =====
void mqttConnect();
void mqttReconnect();
void mqttSubscribeConfig();

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

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

void connectToWiFi() {
  // WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  logger(String("Connected to WiFi: ") + String(ssid));
  logger(String("IP Address: ") + WiFi.localIP().toString());
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
  mqttReconnect();
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    logger("Connecting to MQTT...");
    if (mqttClient.connect("ESP32Client_LCD", mqtt_user, mqtt_password)) {
      logger("Connected to MQTT broker.");
    } else {
      logger(String("Failed, rc=") + String(mqttClient.state()), "WARN");
      delay(2000);
    }
  }
}

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
  delay(10);
}

float medianOfBuffer() {
  float tmp[MEDIAN_N];
  for (int i = 0; i < MEDIAN_N; ++i) tmp[i] = medianBuf[i];
  for (int i = 0; i < MEDIAN_N-1; ++i) for (int j = i+1; j < MEDIAN_N; ++j)
    if (tmp[j] < tmp[i]) { float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
  return tmp[MEDIAN_N/2];
}

void read_ina219() {
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
    logger("Bus Voltage: " + String(ina219.getBusVoltage_V()) + " V");
    logger("Shunt Voltage raw: " + String(shunt_mV) + " mV");
    logger("Shunt Voltage corr: " + String(shunt_corr) + " mV");
    logger("Current (mA, raw): " + String(current_mA));
    logger("Current (mA, median): " + String(currentMed_mA));
    logger("Current (mA, EMA): " + String(currentEMA_mA));
    logger("Water Level: " + String(waterLevel) + " m");
}

void setup() {
  logger("Initializing...");
  espClient.setInsecure(); // disables certificate verification

  Serial.begin(115200);
  connectToWiFi();
  initTimeAndWait();

  mqttConnect();

  memory.begin("myApp", false); 

  ina219_init();
  // calibrateShuntAtKnownCurrent();

  shuntOffset_mV = memory.getFloat("shuntOffset_mV", 0.0);
  logger("Loaded shuntOffset_mV from memory: " + String(shuntOffset_mV,6));
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

  read_ina219();

  delay(2000);
}