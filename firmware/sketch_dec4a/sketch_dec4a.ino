#include <Arduino.h>
#include <Wire.h>

const uint8_t SDA_PIN = 4;  // D2 on Freenove ESP8266
const uint8_t SCL_PIN = 5;  // D1 on Freenove ESP8266
const unsigned long SCAN_INTERVAL_MS = 5000;
const uint8_t RED_LED_PIN = 12;   // D6
const uint8_t GREEN_LED_PIN = 13; // D7

void printKnownDevice(uint8_t address) {
  switch (address) {
    case 0x57:
      Serial.println("  Likely MAX3010x pulse oximeter");
      break;
    case 0x68:
      Serial.println("  Likely MPU6050/MPU9250 IMU");
      break;
    case 0x69:
      Serial.println("  Alternate MPU6050/MPU9250 address");
      break;
    default:
      Serial.println("  Unknown device (check datasheet)");
      break;
  }
}

void scanI2CBus() {
  Serial.println();
  Serial.println("Scanning I2C bus...");

  uint8_t candidates[127];
  uint8_t count = 0;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      candidates[count++] = address;
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices detected.");
  } else {
    Serial.println("Possible active device addresses:");
    for (uint8_t i = 0; i < count; i++) {
      uint8_t address = candidates[i];
      Serial.print("  0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println();
      printKnownDevice(address);
    }
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("I2C sensor address scanner");

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.print("Using SDA pin ");
  Serial.print(SDA_PIN);
  Serial.print(", SCL pin ");
  Serial.println(SCL_PIN);
  
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(200);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);

  scanI2CBus();
}

void loop() {
  static unsigned long lastScan = 0;
  unsigned long now = millis();
  if (now - lastScan >= SCAN_INTERVAL_MS) {
    lastScan = now;
    digitalWrite(RED_LED_PIN, HIGH);
    scanI2CBus();
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(150);
    digitalWrite(GREEN_LED_PIN, LOW);
  }
}