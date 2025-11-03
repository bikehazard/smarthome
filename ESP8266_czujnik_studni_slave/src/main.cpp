#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>

// ======================= DEBUG MACROS =======================
#define DEBUG 0   // <-- set to 0 to disable all Serial prints

#if DEBUG
  #define DEBUG_PRINT(...)     Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)   Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// ======================= UART (SoftwareSerial) =====================
#define RX_PIN D1
#define TX_PIN D2
#define RE_DE_PIN D4
SoftwareSerial uartToMaster(RX_PIN, TX_PIN);  // RX, TX
#define UART_BAUD 9600

// ======================= MS5803 Sensor Setup =====================
#define MS5803_PIN_SDA D6   // ESP8266 SDA
#define MS5803_PIN_SCL D5   // ESP8266 SCL

MS5803 pressureSensor(ADDRESS_HIGH);  // Use ADDRESS_HIGH (0x77) or ADDRESS_LOW (0x76)

// ======================= Timing =====================
const unsigned long readInterval = 2000; // 2 seconds between readings

// ======================= Watchdog Ticker =======================
Ticker wdtTicker;
const unsigned long FEED_WDT_INTERVAL_MS = 2000;
const unsigned long WDT_TIMEOUT_MS = 8000;

void IRAM_ATTR feedWatchdog() {
  ESP.wdtFeed(); // feed software watchdog
}

// ======================= Initialization =====================

void connectMS5803() {
  Wire.begin(MS5803_PIN_SDA, MS5803_PIN_SCL);
  delay(100);
  pressureSensor.begin();
}

// ======================= Reading Function =====================

std::pair<float, float> readMS5803() {
  float temp = pressureSensor.getTemperature(CELSIUS, ADC_4096);
  float press = pressureSensor.getPressure(ADC_4096);

  if (isnan(temp) || isnan(press)) {
    DEBUG_PRINTLN("[ERR] Invalid sensor data!");
    uartToMaster.println("[ERR] Invalid sensor data!");
    return std::make_pair(NAN, NAN);
  }

  DEBUG_PRINT("[MS5803] Temp: ");
  DEBUG_PRINT(temp, 2);
  DEBUG_PRINT(" °C  |  Pressure: ");
  DEBUG_PRINT(press, 2);
  DEBUG_PRINTLN(" mbar");

  return std::make_pair(temp, press);
}

// ======================= UART Communication =====================

void sendToMasterESP32(const String& command) {
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(50);
  uartToMaster.println(command);
  uartToMaster.flush();
  delayMicroseconds(100);
  digitalWrite(RE_DE_PIN, LOW);
  DEBUG_PRINTLN("[UART] Sent to master ESP32: " + command);
}

// ======================= UART Command Handler =====================

void handleSerial() {
  if (uartToMaster.available()) {
    String cmd = uartToMaster.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    DEBUG_PRINTLN("[CMD] Received command: " + cmd);

    String response;
    if (cmd == "T") {
      float temp = pressureSensor.getTemperature(CELSIUS, ADC_4096);
      response = "TEMP=" + String(temp, 2);
    } else if (cmd == "P") {
      float press = pressureSensor.getPressure(ADC_4096);
      response = "PRESS=" + String(press, 2);
    } else if (cmd == "ALL") {
      auto result = readMS5803();
      if (!isnan(result.first)) {
        response = "TEMP=" + String(result.first, 2) + ", PRESS=" + String(result.second, 2);
      } else {
        response = "[ERR] Sensor read failed.";
      }
    } else if (cmd == "RECONNECT") {
      connectMS5803();
      response = "[INFO] Sensor reinitialized.";
    } else if (cmd == "RESET") {
      response = "Rebooting ESP8266...";
      sendToMasterESP32(response);
      delay(500);
      ESP.restart();
      return;
    } else {
      response = "[CMD] Available: T, P, ALL, RECONNECT, RESET";
    }

    if (!response.isEmpty()) {
      sendToMasterESP32(response);
    }
  }
}

// ======================= Setup =====================

void setup() {
  #if DEBUG
    Serial.begin(115200);
  #endif

  // Turn off WiFi to save power
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);

  // Init UART
  uartToMaster.begin(UART_BAUD);
  uartToMaster.setTimeout(200);

  // Init RE/DE pin for RS485
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW); // receive mode
  delay(100);

  // Watchdog setup
  ESP.wdtEnable(WDT_TIMEOUT_MS);
  wdtTicker.attach_ms(FEED_WDT_INTERVAL_MS, feedWatchdog);

  // Show last reset reason
  DEBUG_PRINT("[INFO] Last reset reason: ");
  DEBUG_PRINTLN(ESP.getResetReason());

  DEBUG_PRINTLN("[INFO] Reset reason: " + String(ESP.getResetReason()));
  sendToMasterESP32("[INFO] Reset reason: " + String(ESP.getResetReason()));

  DEBUG_PRINTLN("\n=== MS5803 UART Monitor ===");
  connectMS5803();

  DEBUG_PRINTLN("[INFO] Initialized...");
  sendToMasterESP32("[INFO] Initialized...");
}

// ======================= Loop =====================

void loop() {
  handleSerial();
  ESP.wdtFeed();  // additional safety feed
}
