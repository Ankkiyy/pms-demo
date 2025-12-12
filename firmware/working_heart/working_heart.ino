/*
  Freenove ESP8266 + MAX30105
  Serial-only output (no display, no NeoPixel)
  D2 = SDA (GPIO4), D1 = SCL (GPIO5)
  D6 = RED LED (GPIO12), D7 = GREEN LED (GPIO13)
*/

#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const int D1 = 5;
const int D2 = 4;
const int RED_LED_PIN   = 12;  // D6
const int GREEN_LED_PIN = 13;  // D7

long lastBeat = 0;
float beatsPerMinute = 0.0;
float spO2 = 0.0;

bool isInhaling = false;
bool isExhaling = false;
long irMovingAverage = 0;
const int smoothingFactor = 20;

const long MIN_IR_VALUE = 5000;
bool fingerDetected = false;

// ---------------- PROTOTYPES ----------------
boolean checkForBeat(long irValue);
float calculateSpO2Simple(long redValue, long irValue);

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("Booting Freenove ESP8266 + MAX30105 (serial-only) ...");

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);

  Wire.begin(D2, D1); // SDA, SCL

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR: MAX30105 not found. Check wiring and 3.3V power.");
    while (1) {
      digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
      delay(500);
    }
  }

  // sensible defaults
  byte ledBrightness = 0x1F;   // 31
  byte sampleAverage = 8;
  byte ledMode = 3;            // Red + IR + Green (use 3 to keep compatibility)
  int sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableDIETEMPRDY();

  Serial.println("MAX30105 initialised. Place finger on sensor and keep still.");
}

// ---------------- LOOP ----------------
void loop() {
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // finger detection
  fingerDetected = (irValue > MIN_IR_VALUE);

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print("  RED=");
  Serial.print(redValue);
  Serial.print("  Finger=");
  Serial.println(fingerDetected ? "YES" : "NO");

  if (fingerDetected) {
    // BPM detection
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      if (delta > 300) {
        lastBeat = millis();
        float bpm = 60.0 / (delta / 1000.0);
        if (bpm > 40 && bpm < 180) {
          beatsPerMinute = bpm;
          Serial.print("Detected beat. BPM=");
          Serial.println(beatsPerMinute);
        }
      }
    }

    // SpO2
    spO2 = calculateSpO2Simple(redValue, irValue);
    float temperatureC = particleSensor.readTemperature();

    // simple inhale/exhale detection (based on IR moving average)
    irMovingAverage = (irMovingAverage * (smoothingFactor - 1) + irValue) / smoothingFactor;

    if (irValue < irMovingAverage - 300) {
      if (!isInhaling) {
        isInhaling = true;
        isExhaling = false;
        Serial.println("Respiration: Inhale");
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(RED_LED_PIN, LOW);
      }
    } else if (irValue > irMovingAverage + 300) {
      if (!isExhaling) {
        isExhaling = true;
        isInhaling = false;
        Serial.println("Respiration: Exhale");
        digitalWrite(RED_LED_PIN, HIGH);
        digitalWrite(GREEN_LED_PIN, LOW);
      }
    }

    // periodic status print (BPM, SpO2, Temp)
    Serial.print("BPM: ");
    if (beatsPerMinute > 0.0) Serial.println(beatsPerMinute);
    else Serial.println("N/A");

    Serial.print("SpO2: ");
    Serial.print(spO2, 1);
    Serial.println(" %");

    Serial.print("Temp(C): ");
    Serial.println(temperatureC, 1);
  } else {
    // no finger: LEDs off, reset respiration flags
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    isInhaling = false;
    isExhaling = false;
    Serial.println("No finger detected. Place finger on sensor.");
  }

  Serial.println("-------------------------------");
  delay(700); // adjust loop pace as needed
}

// ---------------- checkForBeat ----------------
// Simple derivative-based beat detection used in original sketch
boolean checkForBeat(long irValue) {
  static long prevIrValue = 0;
  static boolean prevState = false;

  boolean beatDetected = (irValue > prevIrValue + 20) && prevState && (millis() - lastBeat > 300);
  prevIrValue = irValue;
  prevState = (irValue > MIN_IR_VALUE);

  return beatDetected;
}

// ---------------- calculateSpO2Simple ----------------
float calculateSpO2Simple(long redValue, long irValue) {
  if (irValue == 0) return 0.0;
  float ratio = (float)redValue / (float)irValue;
  float spO2 = 110.0 - (25.0 * ratio);
  return constrain(spO2, 0.0, 100.0);
}