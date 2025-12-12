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
#include <math.h>
#include <string.h>
#include <DHT.h>
#include "MAX30105.h"
#include "heartRate.h"

constexpr uint8_t MPU_ADDR_PRIMARY = 0x68;
constexpr uint8_t MPU_ADDR_SECONDARY = 0x69;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

constexpr float GRAVITY_MPS2 = 9.80665f;
constexpr float RAD_PER_DEGREE = PI / 180.0f;
constexpr float DEG_PER_RAD = 180.0f / PI;

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

// Environment (DHT22)
const uint8_t DHT_PIN = 2;  // D4
const uint8_t DHT_TYPE = DHT22;
DHT dht(DHT_PIN, DHT_TYPE);

float ambientTempC = NAN;
float ambientHumidity = NAN;
unsigned long lastEnvReadMs = 0;
const unsigned long envReadIntervalMs = 5000;
const float envSmoothingAlpha = 0.2f;

// Motion (MPU6500 class IMU)
const int MPU_INT_PIN = 16;  // D0 optional interrupt
uint8_t imuAddress = 0;
uint8_t imuWhoAmI = 0;
bool mpuReady = false;
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
float imuTemperatureC = NAN;
float pitchDegrees = NAN;
float rollDegrees = NAN;
String postureState = "unknown";
float activityLevel = 0.0f;
String activityState = "sleeping";
uint32_t stepsWalked = 0;
unsigned long lastStepUpdateMs = 0;
bool fallDetected = false;
const float FALL_LOWER_G = 0.5f;
const float FALL_UPPER_G = 2.8f;

// Wi-Fi and backend
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
const bool USE_DUMMY_DATA = false;

unsigned long lastWiFiAttemptMs = 0;
const unsigned long wifiRetryBackoffMs = 5000;
bool hasWiFiEverConnected = false;

float lastSkinTempC = NAN;

// ---------------- PROTOTYPES ----------------
boolean checkForBeat(long irValue);
float calculateSpO2Simple(long redValue, long irValue);
void connectWiFi();
void maintainWiFiConnection();
bool ensureBackend();
bool discoverBackend();
String isoTimestamp();
bool postJson(const String &url, const String &payload);
void sendVitalsToBackend(float skinTempC);
String jsonFloat(float value, uint8_t decimals = 2);
const char *jsonBool(bool value);
const char *decodeWhoAmI(uint8_t value);
bool writeRegister(uint8_t address, uint8_t reg, uint8_t value);
bool readRegisters(uint8_t address, uint8_t reg, uint8_t *buffer, size_t length);
bool readWhoAmI(uint8_t address, uint8_t &value);
bool detectImu(uint8_t &address, uint8_t &whoAmI);
bool initMpu6500(uint8_t address);
bool readMpu6500(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &tempC);
void scanI2CBus();
void updateEnvironment(unsigned long now);
void updateMotion(unsigned long now);

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
  Wire.setClock(100000);

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

  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  dht.begin();
  Serial.println("DHT22 initialised.");

  connectWiFi();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  discoverBackend();

  scanI2CBus();

  Serial.println("Initialising MPU6500/ICM-class IMU ...");
  if (!detectImu(imuAddress, imuWhoAmI)) {
    Serial.println("WARNING: MPU6500-class IMU not detected on I2C bus.");
    mpuReady = false;
  } else {
    Serial.print("WHO_AM_I (0x");
    if (imuAddress < 16) {
      Serial.print('0');
    }
    Serial.print(imuAddress, HEX);
    Serial.print(") = 0x");
    if (imuWhoAmI < 16) {
      Serial.print('0');
    }
    Serial.print(imuWhoAmI, HEX);
    Serial.print(" -> ");
    Serial.println(decodeWhoAmI(imuWhoAmI));

    if (!initMpu6500(imuAddress)) {
      Serial.println("WARNING: MPU6500 initialisation failed.");
      mpuReady = false;
    } else {
      mpuReady = true;
      Serial.print("MPU6500 initialised at 0x");
      Serial.println(imuAddress, HEX);
    }
  }
}

// ---------------- LOOP ----------------
void loop() {
  unsigned long now = millis();

  maintainWiFiConnection();
  updateEnvironment(now);
  updateMotion(now);

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
    lastSkinTempC = temperatureC;
  } else {
    // no finger: LEDs off, reset respiration flags
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    isInhaling = false;
    isExhaling = false;
    Serial.println("No finger detected. Place finger on sensor.");
    lastSkinTempC = NAN;
  }

  Serial.println("-------------------------------");
  sendVitalsToBackend(lastSkinTempC);
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

// ---------------- Additional helpers ----------------

void updateEnvironment(unsigned long now) {
  if (now - lastEnvReadMs < envReadIntervalMs) {
    return;
  }
  lastEnvReadMs = now;
  float newTemp = dht.readTemperature();
  float newHumidity = dht.readHumidity();

  if (!isnan(newTemp)) {
    if (isnan(ambientTempC)) {
      ambientTempC = newTemp;
    } else {
      ambientTempC = (1.0f - envSmoothingAlpha) * ambientTempC + envSmoothingAlpha * newTemp;
    }
  }

  if (!isnan(newHumidity)) {
    if (isnan(ambientHumidity)) {
      ambientHumidity = newHumidity;
    } else {
      ambientHumidity = (1.0f - envSmoothingAlpha) * ambientHumidity + envSmoothingAlpha * newHumidity;
    }
  }
}

void updateMotion(unsigned long now) {
  if (!mpuReady || (now - lastMotionReadMs < motionReadIntervalMs)) {
    fallDetected = false;
    return;
  }

  lastMotionReadMs = now;

  float imuTempC = 0.0f;
  if (readMpu6500(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imuTempC)) {
    motionValid = true;
    imuTemperatureC = imuTempC;
    accelMagnitudeG = sqrtf((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ)) / GRAVITY_MPS2;

    pitchDegrees = atan2f(-accelX, sqrtf((accelY * accelY) + (accelZ * accelZ))) * DEG_PER_RAD;
    rollDegrees = atan2f(accelY, accelZ) * DEG_PER_RAD;

    float deltaG = fabsf(accelMagnitudeG - 1.0f);
    activityLevel = constrain(deltaG * 0.8f, 0.0f, 1.0f);

    if (activityLevel > 0.7f) {
      activityState = "exercising";
    } else if (activityLevel > 0.4f) {
      activityState = "walking";
    } else if (activityLevel > 0.15f) {
      activityState = "sitting";
    } else {
      activityState = "sleeping";
    }

    if (fabsf(pitchDegrees) < 20.0f && fabsf(rollDegrees) < 20.0f) {
      postureState = "upright";
    } else if (fabsf(pitchDegrees) > 60.0f) {
      postureState = "lying";
    } else {
      postureState = "leaning";
    }

    if (activityLevel > 0.35f && (now - lastStepUpdateMs) > 900) {
      stepsWalked += 2 + (uint32_t)(activityLevel * 5.0f);
      lastStepUpdateMs = now;
    }
  } else {
    motionValid = false;
  }

  fallDetected = motionValid &&
                 (accelMagnitudeG > FALL_UPPER_G || accelMagnitudeG < FALL_LOWER_G);
}

void connectWiFi() {
  Serial.print("Connecting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40) {
    delay(250);
    Serial.print('.');
    retry++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    hasWiFiEverConnected = true;
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed.");
  }
  lastWiFiAttemptMs = millis();
}

void maintainWiFiConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  unsigned long now = millis();
  if (!hasWiFiEverConnected && now - lastWiFiAttemptMs < wifiRetryBackoffMs) {
    return;
  }

  if (now - lastWiFiAttemptMs < wifiRetryBackoffMs) {
    return;
  }

  Serial.println("WiFi disconnected. Attempting reconnect...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWiFiAttemptMs = now;
}

bool ensureBackend() {
  if (backendBaseUrl.length() > 0) {
    return true;
  }

  unsigned long now = millis();
  if (now - lastDiscoveryAttempt < discoveryIntervalMs) {
    return false;
  }

  return discoverBackend();
}

bool discoverBackend() {
  lastDiscoveryAttempt = millis();

  if (!udp.begin(DISCOVERY_PORT)) {
    Serial.println("UDP begin failed.");
    return false;
  }

  udp.beginPacket(IPAddress(255, 255, 255, 255), DISCOVERY_PORT);
  udp.write(DISCOVERY_REQUEST);
  udp.endPacket();

  unsigned long start = millis();
  while (millis() - start < 1500) {
    int pkt = udp.parsePacket();
    if (pkt > 0) {
      String res = udp.readString();
      if (res.startsWith(DISCOVERY_RESPONSE)) {
        backendBaseUrl = res.substring(strlen(DISCOVERY_RESPONSE));
        backendBaseUrl.trim();
        Serial.print("Backend discovered at ");
        Serial.println(backendBaseUrl);
        return true;
      }
    }
    delay(20);
  }

  Serial.println("Backend discovery timed out.");
  return false;
}

String isoTimestamp() {
  struct tm t;
  if (!getLocalTime(&t, 1000)) {
    return "1970-01-01T00:00:00Z";
  }
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &t);
  return String(buf);
}

bool postJson(const String &url, const String &payload) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  HTTPClient http;
  WiFiClient client;

  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(payload);
  http.end();

  if (code > 0 && code < 300) {
    Serial.println("POST vitals -> success");
    return true;
  }

  Serial.print("POST vitals failed, code=");
  Serial.println(code);
  return false;
}

void sendVitalsToBackend(float skinTempC) {
  unsigned long now = millis();
  if (now - lastSendMs < sendIntervalMs) {
    return;
  }
  lastSendMs = now;

  if (!ensureBackend()) {
    return;
  }

  String url = backendBaseUrl + "/device/" + DEVICE_ID + "/vitals";

  float hr = fingerDetected ? beatsPerMinute : NAN;
  float spo = fingerDetected ? spO2 : NAN;
  float temp = fingerDetected ? skinTempC : NAN;
  float ambientTemp = ambientTempC;
  float humidity = ambientHumidity;
  float activity = activityLevel;
  String activityLabel = activityState;
  uint32_t steps = stepsWalked;
  bool fall = fallDetected;
  bool motion = motionValid;
  float imuTemp = motionValid ? imuTemperatureC : NAN;
  float accelMag = motionValid ? accelMagnitudeG : NAN;
  float pitch = motionValid ? pitchDegrees : NAN;
  float roll = motionValid ? rollDegrees : NAN;
  String posture = postureState.length() ? postureState : "unknown";

  static float lastHr = 78.0f;
  static float lastSpo = 98.0f;
  static float lastTemp = 36.6f;
  static float lastAmbient = 24.0f;
  static float lastHumidity = 45.0f;
  static float lastActivity = 0.05f;
  static String lastActivityLabel = "sleeping";
  static uint32_t lastSteps = 0;
  static bool lastFall = false;
  static bool lastMotion = false;
  static float lastImuTemp = NAN;
  static float lastAccelMag = NAN;
  static float lastPitch = NAN;
  static float lastRoll = NAN;
  static String lastPosture = "unknown";

  if (USE_DUMMY_DATA) {
    hr = 78.0f;
    spo = 98.0f;
    temp = 36.6f;
    ambientTemp = 24.0f;
    humidity = 45.0f;
    activity = 0.32f;
    activityLabel = "walking";
    steps = 2300;
    fall = false;
    motion = true;
    imuTemp = 32.0f;
    accelMag = 1.05f;
    pitch = 5.0f;
    roll = 2.0f;
    posture = "upright";
  } else {
    if (!isnan(hr)) lastHr = hr;
    if (!isnan(spo)) lastSpo = spo;
    if (!isnan(temp)) lastTemp = temp;
    if (!isnan(ambientTemp)) lastAmbient = ambientTemp;
    if (!isnan(humidity)) lastHumidity = humidity;
    if (!isnan(activity)) lastActivity = activity;
    if (!isnan(imuTemp)) lastImuTemp = imuTemp;
    if (!isnan(accelMag)) lastAccelMag = accelMag;
    if (!isnan(pitch)) lastPitch = pitch;
    if (!isnan(roll)) lastRoll = roll;
    lastActivityLabel = activityLabel;
    lastSteps = steps;
    lastFall = fall;
    lastMotion = motion;
    lastPosture = posture;

    hr = lastHr;
    spo = lastSpo;
    temp = lastTemp;
    ambientTemp = lastAmbient;
    humidity = lastHumidity;
    activity = lastActivity;
    activityLabel = lastActivityLabel;
    steps = lastSteps;
    fall = lastFall;
    motion = lastMotion;
    imuTemp = lastImuTemp;
    accelMag = lastAccelMag;
    pitch = lastPitch;
    roll = lastRoll;
    posture = lastPosture;
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
  payload += "\"fall_detected\":";
  payload += jsonBool(fall);
  payload += ",";
  payload += "\"motion_valid\":";
  payload += jsonBool(motion);
  payload += ",";
  payload += "\"imu_temperature\":" + jsonFloat(imuTemp) + ",";
  payload += "\"accel_magnitude_g\":" + jsonFloat(accelMag, 3) + ",";
  payload += "\"pitch_deg\":" + jsonFloat(pitch, 2) + ",";
  payload += "\"roll_deg\":" + jsonFloat(roll, 2) + ",";
  payload += "\"posture\":\"" + posture + "\"";
  payload += "}}";

  if (!postJson(url, payload)) {
    backendBaseUrl = "";
  }
}

String jsonFloat(float value, uint8_t decimals) {
  if (isnan(value) || isinf(value)) {
    return "null";
  }
  return String(value, decimals);
}

const char *jsonBool(bool value) {
  return value ? "true" : "false";
}

const char *decodeWhoAmI(uint8_t value) {
  switch (value) {
    case 0x70:
      return "MPU6500";
    case 0x68:
      return "MPU6050/ICM";
    case 0x71:
      return "MPU9250";
    case 0x73:
      return "MPU9255";
    default:
      return "Unknown";
  }
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  size_t received = Wire.requestFrom(address, static_cast<uint8_t>(length));
  if (received != length) {
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool readWhoAmI(uint8_t address, uint8_t &value) {
  return readRegisters(address, REG_WHO_AM_I, &value, 1);
}

bool detectImu(uint8_t &address, uint8_t &whoAmI) {
  const uint8_t candidates[2] = {MPU_ADDR_PRIMARY, MPU_ADDR_SECONDARY};
  for (uint8_t i = 0; i < 2; ++i) {
    uint8_t candidate = candidates[i];
    uint8_t value = 0;
    if (readWhoAmI(candidate, value)) {
      if (value == 0x70 || value == 0x68 || value == 0x71 || value == 0x73) {
        address = candidate;
        whoAmI = value;
        return true;
      }
    }
  }
  return false;
}

bool initMpu6500(uint8_t address) {
  if (!writeRegister(address, REG_PWR_MGMT_1, 0x80)) {
    return false;
  }
  delay(100);
  if (!writeRegister(address, REG_PWR_MGMT_1, 0x01)) {
    return false;
  }
  delay(10);
  if (!writeRegister(address, REG_PWR_MGMT_2, 0x00)) {
    return false;
  }
  if (!writeRegister(address, REG_SMPLRT_DIV, 19)) {
    return false;
  }
  if (!writeRegister(address, REG_CONFIG, 0x03)) {
    return false;
  }
  if (!writeRegister(address, REG_GYRO_CONFIG, 0x08)) {
    return false;
  }
  if (!writeRegister(address, REG_ACCEL_CONFIG, 0x08)) {
    return false;
  }
  if (!writeRegister(address, REG_ACCEL_CONFIG2, 0x03)) {
    return false;
  }
  return true;
}

bool readMpu6500(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &tempC) {
  uint8_t raw[14] = {0};
  if (!readRegisters(imuAddress, REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
    return false;
  }

  int16_t accelXRaw = static_cast<int16_t>((raw[0] << 8) | raw[1]);
  int16_t accelYRaw = static_cast<int16_t>((raw[2] << 8) | raw[3]);
  int16_t accelZRaw = static_cast<int16_t>((raw[4] << 8) | raw[5]);
  int16_t tempRaw = static_cast<int16_t>((raw[6] << 8) | raw[7]);
  int16_t gyroXRaw = static_cast<int16_t>((raw[8] << 8) | raw[9]);
  int16_t gyroYRaw = static_cast<int16_t>((raw[10] << 8) | raw[11]);
  int16_t gyroZRaw = static_cast<int16_t>((raw[12] << 8) | raw[13]);

  const float accelScale = (4.0f * GRAVITY_MPS2) / 32768.0f;
  const float gyroScale = (500.0f * RAD_PER_DEGREE) / 32768.0f;
  const float tempScale = 1.0f / 333.87f;
  const float tempOffset = 21.0f;

  ax = accelXRaw * accelScale;
  ay = accelYRaw * accelScale;
  az = accelZRaw * accelScale;
  gx = gyroXRaw * gyroScale;
  gy = gyroYRaw * gyroScale;
  gz = gyroZRaw * gyroScale;
  tempC = (tempRaw * tempScale) + tempOffset;

  return true;
}

void scanI2CBus() {
  Serial.println("Scanning for I2C devices...");
  uint8_t found = 0;
  for (uint8_t address = 1; address < 0x7F; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print(" - Device at 0x");
      if (address < 16) {
        Serial.print('0');
      }
      Serial.println(address, HEX);
      found++;
    }
    delay(2);
  }
  if (found == 0) {
    Serial.println("No I2C devices found.");
  }
}