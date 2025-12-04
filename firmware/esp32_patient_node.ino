/**
 * ESP32 patient node firmware skeleton.
 *
 * Replace sensor stubs with real library calls (MAX30102, DS18B20, MPU6050).
 */
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Wi-Fi credentials
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// MQTT credentials
const char* MQTT_HOST = "your.broker.domain";
const int MQTT_PORT = 8883;
const char* MQTT_USER = "patient_node_01";
const char* MQTT_PASS = "strong_password_01";

const char* MQTT_PUB_TOPIC = "patients/patient_node_01/vitals";
const char* MQTT_SUB_TOPIC = "patients/patient_node_01/cmd";

// CA certificate
static const char ca_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
...YOUR_CA_CERT_HERE...
-----END CERTIFICATE-----
)EOF";

const int BUZZER_PIN = 18;
const int LED_PIN = 19;

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);
unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL_MS = 5000;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, msg)) {
    return;
  }

  const char* cmd = doc["cmd"];
  if (strcmp(cmd, "ALARM_ON") == 0) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
  } else if (strcmp(cmd, "ALARM_OFF") == 0) {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
  }
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    mqttClient.connect("patient_node_01", MQTT_USER, MQTT_PASS);
    mqttClient.subscribe(MQTT_SUB_TOPIC);
  }
}

float readHeartRate() { return 80.0; }
float readSpO2() { return 98.0; }
float readTemperature() { return 36.7; }
bool detectFall() { return false; }

void publishVitals() {
  StaticJsonDocument<512> doc;
  doc["device_id"] = "patient_node_01";
  doc["patient_hash"] = "a7c9f2";

  JsonObject vitals = doc.createNestedObject("vitals");
  vitals["heart_rate"] = readHeartRate();
  vitals["spo2"] = readSpO2();
  vitals["temperature"] = readTemperature();
  vitals["fall_detected"] = detectFall();

  char buffer[512];
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish(MQTT_PUB_TOPIC, buffer, n);
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  connectWiFi();

  secureClient.setCACert(ca_cert);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  connectMQTT();
}

void loop() {
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();

  unsigned long now = millis();
  if (now - lastPublish > PUBLISH_INTERVAL_MS) {
    lastPublish = now;
    publishVitals();
  }
}
