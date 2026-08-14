#include "IMU.h"

IMU::IMU() 
    : mpuAddr(0x68), initialized(false), roll(0.0f), pitch(0.0f),
      gyroX(0.0f), gyroY(0.0f), gyroZ(0.0f),
      accelX(0.0f), accelY(0.0f), accelZ(0.0f),
      gyroXOffset(0.0f), gyroYOffset(0.0f), gyroZOffset(0.0f),
      accelXOffset(0.0f), accelYOffset(0.0f), accelZOffset(0.0f) {}

bool IMU::begin() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz Fast Mode I2C

#if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_AVR)
    Wire.setWireTimeout(3000, true); // 3ms timeout to prevent I2C hangs if IMU disconnected
#endif

    uint8_t addrs[2] = {0x68, 0x69};
    uint8_t foundAddr = 0;

    for (uint8_t i = 0; i < 2; i++) {
        uint8_t addr = addrs[i];
        Wire.beginTransmission(addr);
        Wire.write(0x6B);
        Wire.write(0x00); // Wake up
        if (Wire.endTransmission(true) == 0) {
            delay(20);
            Wire.beginTransmission(addr);
            Wire.write(0x75); // WHO_AM_I
            Wire.endTransmission(false);
            Wire.requestFrom(addr, (uint8_t)1);
            if (Wire.available()) {
                uint8_t id = Wire.read();
                if (id == 0x68 || id == 0x69 || id == addr) {
                    foundAddr = addr;
                    break;
                }
            }
        }
    }

    if (foundAddr == 0) {
        initialized = false;
        return false;
    }

    mpuAddr = foundAddr;

    // Configure Gyro Full Scale Range (±500 deg/s -> GYRO_CONFIG = 0x08)
    writeRegister(0x1B, 0x08);

    // Configure Accel Full Scale Range (±8g -> ACCEL_CONFIG = 0x10)
    writeRegister(0x1C, 0x10);

    // Configure Digital Low Pass Filter (DLPF ~ 42Hz cutoff -> CONFIG = 0x03)
    writeRegister(0x1A, 0x03);

    initialized = true;

    // Calibrate sensor offsets while still
    calibrate();

    return true;
}

void IMU::writeRegister(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(mpuAddr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission(true);
}

void IMU::readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire.beginTransmission(mpuAddr);
    Wire.write(0x3B); // Starting register for Accel X High Byte
    Wire.endTransmission(false);
    Wire.requestFrom(mpuAddr, (uint8_t)14); // 14 bytes: 6 accel, 2 temp, 6 gyro

    if (Wire.available() >= 14) {
        ax = (Wire.read() << 8) | Wire.read();
        ay = (Wire.read() << 8) | Wire.read();
        az = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Skip temperature bytes
        gx = (Wire.read() << 8) | Wire.read();
        gy = (Wire.read() << 8) | Wire.read();
        gz = (Wire.read() << 8) | Wire.read();
    }
}

void IMU::calibrate() {
    if (!initialized) return;

    int32_t sumGx = 0, sumGy = 0, sumGz = 0;
    int32_t sumAx = 0, sumAy = 0, sumAz = 0;
    const int samples = 500;

    for (int i = 0; i < samples; i++) {
        int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
        readRawData(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

        sumAx += rawAx;
        sumAy += rawAy;
        sumAz += rawAz;
        sumGx += rawGx;
        sumGy += rawGy;
        sumGz += rawGz;
        delay(2);
    }

    gyroXOffset = (float)sumGx / samples;
    gyroYOffset = (float)sumGy / samples;
    gyroZOffset = (float)sumGz / samples;

    accelXOffset = (float)sumAx / samples;
    accelYOffset = (float)sumAy / samples;
    accelZOffset = ((float)sumAz / samples) - 4096.0f; // 4096 LSB/g at ±8g (1g gravity expected on Z)
}

void IMU::update(float dt) {
    if (!initialized || dt <= 0.0f) return;

    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
    readRawData(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

    // Apply calibration offsets & scale factors
    // ±500 deg/s sensitivity: 65.5 LSB / (deg/s)
    gyroX = ((float)rawGx - gyroXOffset) / 65.5f;
    gyroY = ((float)rawGy - gyroYOffset) / 65.5f;
    gyroZ = ((float)rawGz - gyroZOffset) / 65.5f;

    // ±8g sensitivity: 4096.0 LSB / g
    accelX = ((float)rawAx - accelXOffset) / 4096.0f;
    accelY = ((float)rawAy - accelYOffset) / 4096.0f;
    accelZ = ((float)rawAz - accelZOffset) / 4096.0f;

    // Calculate accelerometer pitch and roll angle (degrees)
    float accelRoll  = atan2(accelY, accelZ) * RAD_TO_DEG;
    float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;

    // Complementary Filter: 98% Gyro integration + 2% Accelerometer vector correction
    roll  = 0.98f * (roll + gyroX * dt) + 0.02f * accelRoll;
    pitch = 0.98f * (pitch + gyroY * dt) + 0.02f * accelPitch;
}
