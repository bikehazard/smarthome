#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>

// ======================= UART (SoftwareSerial) =====================
#define RX_PIN D1
#define TX_PIN D2
#define RE_DE_PIN D4
SoftwareSerial uartToMaster(RX_PIN, TX_PIN);  // RX, TX
#define UART_BAUD 9600

// ======================= MS5803 Sensor Setup =====================
#define MS5803_PIN_SDA D5   // ESP8266 SDA
#define MS5803_PIN_SCL D6   // ESP8266 SCL

MS5803 pressureSensor(ADDRESS_HIGH);  // Use ADDRESS_HIGH (0x77) or ADDRESS_LOW (0x76)

// ======================= Timing =====================
unsigned long lastRead = 0;
const unsigned long readInterval = 2000; // 2 seconds between readings

// ======================= Initialization =====================

void connectMS5803() {
  Serial.println("[INFO] Initializing I2C and MS5803 sensor...");
  Wire.begin(MS5803_PIN_SDA, MS5803_PIN_SCL);
  delay(100);
  pressureSensor.begin();
}

// ======================= Reading Function =====================

std::pair<float, float> readMS5803() {
  float temp = pressureSensor.getTemperature(CELSIUS, ADC_4096);
  float press = pressureSensor.getPressure(ADC_4096);

  if (isnan(temp) || isnan(press)) {
    Serial.print("[ERR] Invalid sensor data!");
    uartToMaster.println("[ERR] Invalid sensor data!");
    return std::make_pair(NAN, NAN);
  }

  Serial.print("[MS5803] Temp: ");
  Serial.print(temp, 2);
  Serial.print(" °C  |  Pressure: ");
  Serial.print(press, 2);
  Serial.println(" mbar");

  return std::make_pair(temp, press);
}

void sendToMasterESP32(const String& command) {
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(50);
  uartToMaster.println(command);
  uartToMaster.flush();
  delayMicroseconds(100);
  digitalWrite(RE_DE_PIN, LOW);
  Serial.println("[UART] Sent to master ESP32: " + command);
}

// ======================= UART Command Handler =====================
void handleSerial() {
  if (uartToMaster.available()) {
    String cmd = uartToMaster.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    Serial.println("[CMD] Received command: " + cmd);

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
      response = "[CMD] Rebooting ESP8266...";
      sendToMasterESP32(response);
      delay(500);
      ESP.restart();
      return;
    } else {
      response = "[CMD] Available: T, P, ALL, RECONNECT, RESET";
    }

    if(!response.isEmpty())
    {
      sendToMasterESP32(response);
    }
  }
}

// ======================= Setup =====================
void setup() {
  // uart.println("[INFO] Starting up...");

  // turn off WIFI to save power
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);

  // init serial and UART
  Serial.begin(115200);
  uartToMaster.begin(UART_BAUD);
  uartToMaster.setTimeout(200);

  // init RE/DE pin for RS485
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW); // Set RE/DE low to receive
  delay(100);

  Serial.println("\n=== MS5803 UART Monitor ===");
  connectMS5803();
}

// ======================= Loop =====================
void loop() {
  handleSerial();

  // readMS5803();
  // delay(readInterval);
}
