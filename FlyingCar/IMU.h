#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"

class IMU {
public:
    IMU();
    bool begin();
    void update(float dt);
    void calibrate();

    // Getter methods
    float getRoll() const { return roll; }         // Degrees
    float getPitch() const { return pitch; }       // Degrees
    float getGyroX() const { return gyroX; }       // deg/s (Roll rate)
    float getGyroY() const { return gyroY; }       // deg/s (Pitch rate)
    float getGyroZ() const { return gyroZ; }       // deg/s (Yaw rate)

    bool isInitialized() const { return initialized; }

private:
    uint8_t mpuAddr;

    bool initialized;

    // Filtered estimates
    float roll;
    float pitch;

    // Calibrated rate readings
    float gyroX;
    float gyroY;
    float gyroZ;

    // Raw accel readings
    float accelX;
    float accelY;
    float accelZ;

    // Calibration offsets
    float gyroXOffset;
    float gyroYOffset;
    float gyroZOffset;
    float accelXOffset;
    float accelYOffset;
    float accelZOffset;

    void writeRegister(uint8_t reg, uint8_t data);
    void readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);
};

#endif // IMU_H
