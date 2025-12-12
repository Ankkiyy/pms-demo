/*
  Freenove ESP8266 + MAX30105 + MPU9250 + DHT22
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

#include <DHT.h>
#include "MAX30105.h"
#include "heartRate.h"

// ---- MPU9250 LIBRARY ----
#include "mpu9250.h"

// ---------------- SENSORS & GLOBALS ----------------
MAX30105 particleSensor;
long lastIrSample = 0;
long lastRedSample = 0;

// an MPU9250 object with MPU-9250 at I2C addr 0x68
// bfs::Mpu9250 imu(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

// an MPU9250 object with MPU-9250 at I2C addr 0x69
bfs::Mpu9250 imu(&Wire, bfs::Mpu9250::I2C_ADDR_SEC);

const uint8_t DHT_PIN = 2;  // D4
const uint8_t DHT_TYPE = DHT22;
const int MPU_INT_PIN = 16;  // D0 (optional interrupt pin, not strictly needed for basic reads)

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

// Toggle between real sensor values and demo placeholders
const bool USE_DUMMY_DATA = false;

const int D1 = 5;              // SCL
const int D2 = 4;              // SDA
const int RED_LED_PIN = 12;    // D6
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

// MPU / motion
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

float activityLevel = 0.0f;
String activityState = "sleeping";
uint32_t stepsWalked = 0;

const float FALL_LOWER_G = 0.5f;
const float FALL_UPPER_G = 2.8f;

// ---------------- PROTOTYPES ----------------
float calculateSpO2Simple(long redValue, long irValue);
String jsonFloat(float value, uint8_t decimals = 2);
void connectWiFi();
bool discoverBackend();
String isoTimestamp();
bool postJson(const String &url, const String &payload);
void sendVitalsToBackend(float temperatureC);

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("Booting Freenove ESP8266 + MAX30105 + MPU9250 (serial-only) ...");

  dht.begin();
  Serial.println("DHT22 initialised.");

  connectWiFi();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  discoverBackend();

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);

  pinMode(MPU_INT_PIN, INPUT_PULLUP);  // optional, not required for polling

  // I2C init with custom pins on ESP8266
  Wire.begin(D2, D1);  // SDA, SCL

  // --------- MPU9250 INIT ----------
  Serial.println("Initialising MPU9250 ...");

  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  if (!imu.Begin()) {
    Serial.println("WARNING: MPU9250 not found or failed init.");
    mpuReady = false;
  } else {
    mpuReady = true;
    imu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_4G);
    imu.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_500DPS);
    imu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_20HZ);
    imu.ConfigSrd(19);  // ~50 Hz sample rate
    Serial.println("MPU9250 initialised.");
  }


  // --------- MAX30105 (Pulse Oximeter) INIT ----------
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR: MAX30105 not found. Check wiring and 3.3V power.");
    while (1) {
      digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
      delay(500);
    }
  }

  // sensible defaults
  byte ledBrightness = 0x1F;  // 31
  byte sampleAverage = 8;
  byte ledMode = 3;  // Red + IR + Green
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

  // Read MAX30105
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();
  float temperatureC = particleSensor.readTemperature();
  lastIrSample = irValue;
  lastRedSample = redValue;

  // --------- ENVIRONMENT (DHT22) ----------
  if (now - lastEnvReadMs >= envReadIntervalMs) {
    lastEnvReadMs = now;
    float newTemp = dht.readTemperature();
    float newHumidity = dht.readHumidity();
    if (!isnan(newTemp)) {
      ambientTempC = newTemp;
    }
    if (!isnan(newHumidity)) {
      ambientHumidity = newHumidity;
    }
  }

  // --------- MPU9250 READ ----------  
  if (mpuReady && (now - lastMotionReadMs >= motionReadIntervalMs)) {
    lastMotionReadMs = now;
    if (imu.Read()) {
      motionValid = true;
      accelX = imu.accel_x_mps2();
      accelY = imu.accel_y_mps2();
      accelZ = imu.accel_z_mps2();
      gyroX  = imu.gyro_x_radps();
      gyroY  = imu.gyro_y_radps();
      gyroZ  = imu.gyro_z_radps();
      accelMagnitudeG = sqrt((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ)) / 9.80665f;
    }
  }

  fallDetected = motionValid &&
                 (accelMagnitudeG > FALL_UPPER_G || accelMagnitudeG < FALL_LOWER_G);

  if (motionValid) {
    activityLevel = constrain(fabs(accelMagnitudeG - 1.0f) * 0.8f, 0.0f, 1.0f);
    if (activityLevel > 0.7f) {
      activityState = "exercising";
    } else if (activityLevel > 0.4f) {
      activityState = "walking";
    } else if (activityLevel > 0.15f) {
      activityState = "sitting";
    } else {
      activityState = "sleeping";
    }

    // crude step estimate based on sustained motion
    static unsigned long lastStepUpdate = 0;
    if (activityLevel > 0.35f && now - lastStepUpdate > 900) {
      stepsWalked += 2 + (uint32_t)(activityLevel * 5);
      lastStepUpdate = now;
    }
  }

  // finger detection
  fingerDetected = (irValue > MIN_IR_VALUE);

  // BPM detection
  if (fingerDetected && checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    if (delta > 300) {
      float bpm = 60.0f / (delta / 1000.0f);
      if (bpm > 35.0f && bpm < 195.0f) {
        beatsPerMinute = bpm;
      }
    }
  }

  // SpO2
  spO2 = calculateSpO2Simple(redValue, irValue);

  // respiration
  irMovingAverage = (irMovingAverage * (smoothingFactor - 1) + irValue) / smoothingFactor;
  if (irValue < irMovingAverage - 300) {
    isInhaling = true;
    isExhaling = false;
  } else if (irValue > irMovingAverage + 300) {
    isExhaling = true;
    isInhaling = false;
  }

  sendVitalsToBackend(temperatureC);
}


float calculateSpO2Simple(long redValue, long irValue) {
  if (irValue == 0) return 0.0;
  float ratio = (float)redValue / (float)irValue;
  float spo2 = 110.0 - (25.0 * ratio);
  return constrain(spo2, 0.0, 100.0);
}

String jsonFloat(float value, uint8_t decimals) {
  if (isnan(value) || isinf(value)) return "null";
  return String(value, decimals);
}

void connectWiFi() {
  Serial.print("Connecting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40) {
    delay(250);
    retry++;
  }
  Serial.println();
}

bool discoverBackend() {
  udp.begin(DISCOVERY_PORT);
  udp.beginPacket(IPAddress(255,255,255,255), DISCOVERY_PORT);
  udp.write(DISCOVERY_REQUEST);
  udp.endPacket();

  unsigned long start = millis();
  while (millis() - start < 1500) {
    int pkt = udp.parsePacket();
    if (pkt) {
      String res = udp.readString();
      if (res.startsWith(DISCOVERY_RESPONSE)) {
        backendBaseUrl = res.substring(strlen(DISCOVERY_RESPONSE));
        backendBaseUrl.trim();
        return true;
      }
    }
    delay(20);
  }
  return false;
}

String isoTimestamp() {
  struct tm t;
  if (!getLocalTime(&t, 1000)) return "1970-01-01T00:00:00Z";
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &t);
  return String(buf);
}

bool postJson(const String &url, const String &payload) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  WiFiClient client;

  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(payload);
  http.end();
  return (code > 0 && code < 300);
}

void sendVitalsToBackend(float temperatureC) {
  unsigned long now = millis();
  if (now - lastSendMs < sendIntervalMs) return;
  lastSendMs = now;

  if (!discoverBackend()) return;

  String url = backendBaseUrl + "/device/" + DEVICE_ID + "/vitals";

  float hr = (fingerDetected ? beatsPerMinute : NAN);
  float spo = (fingerDetected ? spO2 : NAN);
  float temp = (fingerDetected ? temperatureC : NAN);
  float ambientTemp = ambientTempC;
  float humidity = ambientHumidity;

  float activity = activityLevel;
  String activityLabel = activityState;
  uint32_t steps = stepsWalked;

  static float lastHr = 78.0f;
  static float lastSpo = 98.0f;
  static float lastTemp = 36.6f;
  static float lastAmbient = 24.0f;
  static float lastHumidity = 45.0f;
  static float lastActivity = 0.05f;
  static String lastActivityLabel = "sleeping";
  static uint32_t lastSteps = 0;

  if (USE_DUMMY_DATA) {
    hr = 78.0f;
    spo = 98.0f;
    temp = 36.6f;
    ambientTemp = 24.0f;
    humidity = 45.0f;
    activity = 0.32f;
    activityLabel = "walking";
    steps = 2300;
  } else {
    if (!isnan(hr)) lastHr = hr;
    if (!isnan(spo)) lastSpo = spo;
    if (!isnan(temp)) lastTemp = temp;
    if (!isnan(ambientTemp)) lastAmbient = ambientTemp;
    if (!isnan(humidity)) lastHumidity = humidity;
    if (!isnan(activity)) lastActivity = activity;
    lastActivityLabel = activityLabel;
    lastSteps = steps;

    hr = lastHr;
    spo = lastSpo;
    temp = lastTemp;
    ambientTemp = lastAmbient;
    humidity = lastHumidity;
    activity = lastActivity;
    activityLabel = lastActivityLabel;
    steps = lastSteps;
  }

  String payload = "{";
  payload += "\"device_id\":\"" + String(DEVICE_ID) + "\",";
  payload += "\"patient_hash\":\"" + String(PATIENT_HASH) + "\",";
  payload += "\"timestamp\":\"" + isoTimestamp() + "\",";
  payload += "\"vitals\":{";
  payload += "\"heart_rate\":" + jsonFloat(hr) + ",";
  payload += "\"spo2\":" + jsonFloat(spo) + ",";
  payload += "\"temperature\":" + jsonFloat(temp) + ",";
  payload += "\"ambient_temperature\":" + jsonFloat(ambientTemp) + ",";
  payload += "\"humidity\":" + jsonFloat(humidity) + ",";
  payload += "\"activity_level\":" + jsonFloat(activity, 3) + ",";
  payload += "\"activity_state\":\"" + activityLabel + "\",";
  payload += "\"steps_walked\":" + String(steps) + ",";
  payload += "\"fall_detected\":" + String(fallDetected ? "true" : "false");
  payload += "}}";

  postJson(url, payload);
}

