/*
  Freenove ESP8266 + MAX30105
  Serial-only output (no display, no NeoPixel)
  D2 = SDA (GPIO4), D1 = SCL (GPIO5)
  D6 = RED LED (GPIO12), D7 = GREEN LED (GPIO13)
*/

#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266HTTPClient.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <DHT.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;
long lastIrSample = 0;
long lastRedSample = 0;

Adafruit_MPU6050 mpu;

const uint8_t DHT_PIN = 2;   // D4
const uint8_t DHT_TYPE = DHT22;
const int MPU_INT_PIN = 16;  // D0 (optional interrupt pin)

// Wi-Fi credentials and backend discovery
const char *WIFI_SSID = "JarvisX";
const char *WIFI_PASSWORD = "121212qw";
const char *DEVICE_ID = "esp32_patient_01";
const char *PATIENT_HASH = "demo-patient";
const unsigned int DISCOVERY_PORT = 4211;
const char *DISCOVERY_REQUEST = "PMS_DISCOVER";
const char *DISCOVERY_RESPONSE = "PMS_BACKEND";

WiFiUDP udp;
String backendBaseUrl = "";
unsigned long lastDiscoveryAttempt = 0;
const unsigned long discoveryIntervalMs = 15000;
unsigned long lastSendMs = 0;
const unsigned long sendIntervalMs = 2000;

const int D1 = 5;
const int D2 = 4;
const int RED_LED_PIN   = 12;  // D6
const int GREEN_LED_PIN = 13;  // D7

DHT dht(DHT_PIN, DHT_TYPE);

long lastBeat = 0;
float beatsPerMinute = 0.0;
float spO2 = 0.0;

bool isInhaling = false;
bool isExhaling = false;
long irMovingAverage = 0;
const int smoothingFactor = 20;

const long MIN_IR_VALUE = 5000;
bool fingerDetected = false;

bool mpuReady = false;
float ambientTempC = NAN;
float ambientHumidity = NAN;
unsigned long lastEnvReadMs = 0;
const unsigned long envReadIntervalMs = 5000;

bool motionValid = false;
unsigned long lastMotionReadMs = 0;
const unsigned long motionReadIntervalMs = 200;
float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;
float accelMagnitudeG = NAN;
bool fallDetected = false;

const float FALL_LOWER_G = 0.5f;
const float FALL_UPPER_G = 2.8f;

// ---------------- PROTOTYPES ----------------
boolean checkForBeat(long irValue);
float calculateSpO2Simple(long redValue, long irValue);
String jsonFloat(float value, uint8_t decimals = 2);
void connectWiFi();
bool discoverBackend();
String isoTimestamp();
bool postJson(const String &url, const String &payload);
void sendVitalsToBackend(float temperatureC);
void sendTelemetry(const String &jsonPayload);

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("Booting Freenove ESP8266 + MAX30105 (serial-only) ...");

  dht.begin();
  Serial.println("DHT22 initialised.");

  connectWiFi();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  discoverBackend();

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);

  pinMode(MPU_INT_PIN, INPUT_PULLUP);

  Wire.begin(D2, D1); // SDA, SCL

  if (mpu.begin()) {
    mpuReady = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 initialised.");
  } else {
    Serial.println("WARNING: MPU6050 not fou  nd. Motion data disabled.");
  }

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
  unsigned long now = millis();
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();
  float temperatureC = particleSensor.readTemperature();
  lastIrSample = irValue;
  lastRedSample = redValue;

  if (now - lastEnvReadMs >= envReadIntervalMs) {
    lastEnvReadMs = now;
    float newTemp = dht.readTemperature();
    float newHumidity = dht.readHumidity();
    if (!isnan(newTemp)) {
      ambientTempC = newTemp;
      Serial.print("Ambient Temp(C): ");
      Serial.println(ambientTempC, 1);
    }
    if (!isnan(newHumidity)) {
      ambientHumidity = newHumidity;
      Serial.print("Humidity(%): ");
      Serial.println(ambientHumidity, 1);
    }
  }

  if (mpuReady && (now - lastMotionReadMs >= motionReadIntervalMs)) {
    lastMotionReadMs = now;
    sensors_event_t accelEvent;
    sensors_event_t gyroEvent;
    sensors_event_t tempEvent;
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    accelX = accelEvent.acceleration.x;
    accelY = accelEvent.acceleration.y;
    accelZ = accelEvent.acceleration.z;
    gyroX = gyroEvent.gyro.x;
    gyroY = gyroEvent.gyro.y;
    gyroZ = gyroEvent.gyro.z;
    accelMagnitudeG = sqrtf((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ)) / 9.80665f;
    motionValid = true;
    Serial.print("Accel (m/s^2): ");
    Serial.print(accelX, 2);
    Serial.print(", ");
    Serial.print(accelY, 2);
    Serial.print(", ");
    Serial.print(accelZ, 2);
    Serial.print("  |  Gyro (deg/s): ");
    Serial.print(gyroX, 2);
    Serial.print(", ");
    Serial.print(gyroY, 2);
    Serial.print(", ");
    Serial.println(gyroZ, 2);
    Serial.print("Accel Magnitude (g): ");
    Serial.println(accelMagnitudeG, 2);
  }

  fallDetected = motionValid &&
                 (accelMagnitudeG > FALL_UPPER_G || accelMagnitudeG < FALL_LOWER_G);
  if (fallDetected) {
    Serial.println("Motion alert: fall threshold exceeded.");
  }

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
  sendVitalsToBackend(temperatureC);
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

String jsonFloat(float value, uint8_t decimals) {
  if (isnan(value) || isinf(value)) {
    return String("null");
  }
  return String(value, decimals);
}

// ---------------- NETWORK HELPERS ----------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    delay(250);
    Serial.print(".");
    retries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to WiFi. Check credentials.");
  }
}

bool discoverBackend() {
  unsigned long now = millis();
  if ((backendBaseUrl.length() > 0) && (now - lastDiscoveryAttempt < discoveryIntervalMs)) {
    return true;
  }

  lastDiscoveryAttempt = now;
  if (!udp.begin(DISCOVERY_PORT)) {
    Serial.println("UDP setup for discovery failed.");
    return false;
  }

  Serial.println("Broadcasting for backend...");
  for (int attempt = 0; attempt < 3; attempt++) {
    udp.beginPacket(IPAddress(255, 255, 255, 255), DISCOVERY_PORT);
    udp.write(DISCOVERY_REQUEST);
    udp.endPacket();

    unsigned long startWait = millis();
    while (millis() - startWait < 1200) {
      int packetSize = udp.parsePacket();
      if (packetSize) {
        String response = udp.readString();
        response.trim();
        Serial.print("Discovery response: ");
        Serial.println(response);

        if (response.startsWith(DISCOVERY_RESPONSE)) {
          backendBaseUrl = response.substring(strlen(DISCOVERY_RESPONSE));
          backendBaseUrl.trim();
          Serial.print("Backend discovered at: ");
          Serial.println(backendBaseUrl);
          return true;
        }
      }
      delay(50);
    }
  }

  Serial.println("Backend discovery failed. Will retry.");
  backendBaseUrl = "";
  return false;
}

String isoTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1000)) {
    return "1970-01-01T00:00:00Z";
  }
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buffer);
}

bool postJson(const String &url, const String &payload) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) return false;
  }

  HTTPClient http;
  WiFiClient client;

  Serial.print("POST ");
  Serial.println(url);
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.POST(payload);

  if (httpCode > 0) {
    Serial.print("Response code: ");
    Serial.println(httpCode);
  } else {
    Serial.print("HTTP POST failed: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
  return httpCode > 0 && httpCode < 300;
}

void sendTelemetry(const String &jsonPayload) {
  if (backendBaseUrl.length() == 0) return;
  String url = backendBaseUrl + "/device/" + DEVICE_ID + "/telemetry";
  postJson(url, jsonPayload);
}

void sendVitalsToBackend(float temperatureC) {
  unsigned long now = millis();
  if (now - lastSendMs < sendIntervalMs) return;
  lastSendMs = now;

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!discoverBackend()) return;

  String url = backendBaseUrl + "/device/" + DEVICE_ID + "/vitals";
  String payload = "{\"device_id\":\"" + String(DEVICE_ID) + "\",";
  float hrValue = (fingerDetected && beatsPerMinute > 0.0f) ? beatsPerMinute : NAN;
  float spo2Value = (fingerDetected && spO2 > 0.0f) ? spO2 : NAN;
  float tempValue = fingerDetected ? temperatureC : NAN;

  payload += "\"patient_hash\":\"" + String(PATIENT_HASH) + "\",";
  payload += "\"timestamp\":\"" + isoTimestamp() + "\",";
  payload += "\"vitals\":{";
  payload += "\"heart_rate\":" + jsonFloat(hrValue, 1) + ",";
  payload += "\"spo2\":" + jsonFloat(spo2Value, 1) + ",";
  payload += "\"temperature\":" + jsonFloat(tempValue, 1) + ",";
  payload += "\"fall_detected\":";
  payload += (fallDetected ? "true" : "false");
  payload += "}}";

  bool ok = postJson(url, payload);

  // also emit flexible telemetry (respiration, raw IR) for debugging or expansion
  String breathState = "steady";
  if (isInhaling) breathState = "inhale";
  else if (isExhaling) breathState = "exhale";

  String telemetry = "{\"respiration\":\"" + breathState + "\"";
  telemetry += ",\"fingerDetected\":";
  telemetry += (fingerDetected ? "true" : "false");
  telemetry += ",\"irMovingAverage\":" + String(irMovingAverage);
  telemetry += ",\"ir\":" + String(lastIrSample);
  telemetry += ",\"red\":" + String(lastRedSample);
  telemetry += ",\"fallDetected\":";
  telemetry += (fallDetected ? "true" : "false");
  if (!isnan(ambientTempC)) {
    telemetry += ",\"ambientTemp\":" + jsonFloat(ambientTempC, 1);
  }
  if (!isnan(ambientHumidity)) {
    telemetry += ",\"humidity\":" + jsonFloat(ambientHumidity, 1);
  }
  if (motionValid) {
    telemetry += ",\"accelX\":" + jsonFloat(accelX, 2);
    telemetry += ",\"accelY\":" + jsonFloat(accelY, 2);
    telemetry += ",\"accelZ\":" + jsonFloat(accelZ, 2);
    telemetry += ",\"gyroX\":" + jsonFloat(gyroX, 2);
    telemetry += ",\"gyroY\":" + jsonFloat(gyroY, 2);
    telemetry += ",\"gyroZ\":" + jsonFloat(gyroZ, 2);
    telemetry += ",\"accelMagnitudeG\":" + jsonFloat(accelMagnitudeG, 2);
  }
  telemetry += "}";

  if (ok) {
    sendTelemetry(telemetry);
  }
}
