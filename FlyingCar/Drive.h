#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "Config.h"

class Drive {
public:
    Drive();
    void begin();
    void setCommand(int16_t forward, int16_t steer);
    void update();
    void stop();

    int16_t getCurrentLeftSpeed() const { return currentLeftSpeed; }
    int16_t getCurrentRightSpeed() const { return currentRightSpeed; }

private:
    int16_t targetLeftSpeed;
    int16_t targetRightSpeed;
    int16_t currentLeftSpeed;
    int16_t currentRightSpeed;

    uint32_t lastUpdateMs;

    void applyMotorHardware(int16_t left, int16_t right);
};

#endif // DRIVE_H
