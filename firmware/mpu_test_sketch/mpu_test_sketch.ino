
#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t SDA_PIN = 4;   // D2 on NodeMCU
constexpr uint8_t SCL_PIN = 5;   // D1 on NodeMCU
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

uint8_t imuAddress = 0;
uint8_t imuWhoAmI = 0;
bool imuReady = false;

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
	const size_t received = Wire.requestFrom(address, static_cast<uint8_t>(length));
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
	for (uint8_t candidate : {MPU_ADDR_PRIMARY, MPU_ADDR_SECONDARY}) {
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
	if (!writeRegister(address, REG_PWR_MGMT_1, 0x80)) {  // reset
		return false;
	}
	delay(100);
	if (!writeRegister(address, REG_PWR_MGMT_1, 0x01)) {  // clock: PLL with X axis gyro
		return false;
	}
	delay(10);
	if (!writeRegister(address, REG_PWR_MGMT_2, 0x00)) {
		return false;
	}
	if (!writeRegister(address, REG_SMPLRT_DIV, 19)) {  // ~50 Hz
		return false;
	}
	if (!writeRegister(address, REG_CONFIG, 0x03)) {
		return false;
	}
	if (!writeRegister(address, REG_GYRO_CONFIG, 0x08)) {  // ±500 dps
		return false;
	}
	if (!writeRegister(address, REG_ACCEL_CONFIG, 0x08)) {  // ±4 g
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
	const int16_t accelX = static_cast<int16_t>((raw[0] << 8) | raw[1]);
	const int16_t accelY = static_cast<int16_t>((raw[2] << 8) | raw[3]);
	const int16_t accelZ = static_cast<int16_t>((raw[4] << 8) | raw[5]);
	const int16_t tempRaw = static_cast<int16_t>((raw[6] << 8) | raw[7]);
	const int16_t gyroX = static_cast<int16_t>((raw[8] << 8) | raw[9]);
	const int16_t gyroY = static_cast<int16_t>((raw[10] << 8) | raw[11]);
	const int16_t gyroZ = static_cast<int16_t>((raw[12] << 8) | raw[13]);

	const float accelScale = (4.0f * GRAVITY_MPS2) / 32768.0f;
	const float gyroScale = (500.0f * RAD_PER_DEGREE) / 32768.0f;
	const float tempScale = 1.0f / 333.87f;
	const float tempOffset = 21.0f;

	ax = accelX * accelScale;
	ay = accelY * accelScale;
	az = accelZ * accelScale;
	gx = gyroX * gyroScale;
	gy = gyroY * gyroScale;
	gz = gyroZ * gyroScale;
	tempC = (tempRaw * tempScale) + tempOffset;
	return true;
}

void scanI2CBus() {
	Serial.println(F("Scanning for I2C devices..."));
	uint8_t found = 0;
	for (uint8_t address = 1; address < 0x7F; ++address) {
		Wire.beginTransmission(address);
		if (Wire.endTransmission() == 0) {
			Serial.print(F(" - Device at 0x"));
			if (address < 16) {
				Serial.print('0');
			}
			Serial.println(address, HEX);
			++found;
		}
		delay(2);
	}
	if (found == 0) {
		Serial.println(F("No devices found."));
	}
}

void setup() {
	Serial.begin(115200);
	delay(500);
	Serial.println();
	Serial.println(F("MPU6500 smoke test"));

	Wire.begin(SDA_PIN, SCL_PIN);
	Wire.setClock(100000);
	Serial.println(F("I2C clock set to 100 kHz"));

	scanI2CBus();

	if (!detectImu(imuAddress, imuWhoAmI)) {
		Serial.println(F("ERROR: No MPU6500-class device detected."));
		return;
	}

	Serial.print(F("WHO_AM_I (0x"));
	Serial.print(imuAddress, HEX);
	Serial.print(F(") = 0x"));
	if (imuWhoAmI < 16) {
		Serial.print('0');
	}
	Serial.print(imuWhoAmI, HEX);
	Serial.print(F(" -> "));
	Serial.println(decodeWhoAmI(imuWhoAmI));

	if (!initMpu6500(imuAddress)) {
		Serial.println(F("ERROR: Failed to initialize MPU6500."));
		return;
	}

	imuReady = true;
	Serial.println(F("Initialization complete. Reading sensor data..."));
	Serial.flush();
}

void loop() {
	static unsigned long lastPrint = 0;
	const unsigned long now = millis();

	if (!imuReady) {
		static unsigned long lastWarn = 0;
		if (now - lastWarn >= 1000) {
			lastWarn = now;
			Serial.println(F("MPU6500 not ready. Check wiring/power and reset."));
		}
		delay(50);
		return;
	}

	if (now - lastPrint < 200) {
		return;
	}
	lastPrint = now;

	float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, tempC = 0;
	if (!readMpu6500(ax, ay, az, gx, gy, gz, tempC)) {
		Serial.println(F("Failed to read sample."));
		return;
	}

	Serial.println(F("----------------------------------------"));
	Serial.print(F("Accel (m/s^2): "));
	Serial.print(ax, 3);
	Serial.print(F(", "));
	Serial.print(ay, 3);
	Serial.print(F(", "));
	Serial.println(az, 3);

	Serial.print(F("Gyro (rad/s): "));
	Serial.print(gx, 3);
	Serial.print(F(", "));
	Serial.print(gy, 3);
	Serial.print(F(", "));
	Serial.println(gz, 3);

	Serial.print(F("Die Temp (°C): "));
	Serial.println(tempC, 2);
}
