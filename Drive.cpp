#include "Drive.h"

Drive::Drive()
    : targetLeftSpeed(0), targetRightSpeed(0),
      currentLeftSpeed(0), currentRightSpeed(0),
      lastUpdateMs(0) {}

void Drive::begin() {
    pinMode(PIN_DRIVE_IN1, OUTPUT);
    pinMode(PIN_DRIVE_IN2, OUTPUT);
    pinMode(PIN_DRIVE_IN3, OUTPUT);
    pinMode(PIN_DRIVE_IN4, OUTPUT);
    pinMode(PIN_DRIVE_ENA, OUTPUT);
    pinMode(PIN_DRIVE_ENB, OUTPUT);

    stop();
}

void Drive::setCommand(int16_t forward, int16_t steer) {
    // Clamp input values to valid [-255, 255] range
    forward = constrain(forward, (int16_t)-255, (int16_t)255);
    steer   = constrain(steer, (int16_t)-255, (int16_t)255);

    // Skid/Differential steering mixing logic
    int16_t left  = forward + steer;
    int16_t right = forward - steer;

    targetLeftSpeed  = constrain(left, (int16_t)-255, (int16_t)255);
    targetRightSpeed = constrain(right, (int16_t)-255, (int16_t)255);
}

void Drive::stop() {
    targetLeftSpeed  = 0;
    targetRightSpeed = 0;
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    applyMotorHardware(0, 0);
}

void Drive::update() {
    uint32_t now = millis();
    if (now - lastUpdateMs < 20) return; // Execute ramping tick every 20ms
    lastUpdateMs = now;

    // Ramping logic for Left Motor
    if (currentLeftSpeed < targetLeftSpeed) {
        currentLeftSpeed += DRIVE_RAMP_STEP;
        if (currentLeftSpeed > targetLeftSpeed) currentLeftSpeed = targetLeftSpeed;
    } else if (currentLeftSpeed > targetLeftSpeed) {
        currentLeftSpeed -= DRIVE_RAMP_STEP;
        if (currentLeftSpeed < targetLeftSpeed) currentLeftSpeed = targetLeftSpeed;
    }

    // Ramping logic for Right Motor
    if (currentRightSpeed < targetRightSpeed) {
        currentRightSpeed += DRIVE_RAMP_STEP;
        if (currentRightSpeed > targetRightSpeed) currentRightSpeed = targetRightSpeed;
    } else if (currentRightSpeed > targetRightSpeed) {
        currentRightSpeed -= DRIVE_RAMP_STEP;
        if (currentRightSpeed < targetRightSpeed) currentRightSpeed = targetRightSpeed;
    }

    applyMotorHardware(currentLeftSpeed, currentRightSpeed);
}

void Drive::applyMotorHardware(int16_t left, int16_t right) {
    // Left Motor Direction & PWM
    if (left > 0) {
        digitalWrite(PIN_DRIVE_IN1, HIGH);
        digitalWrite(PIN_DRIVE_IN2, LOW);
        analogWrite(PIN_DRIVE_ENA, left);
    } else if (left < 0) {
        digitalWrite(PIN_DRIVE_IN1, LOW);
        digitalWrite(PIN_DRIVE_IN2, HIGH);
        analogWrite(PIN_DRIVE_ENA, -left);
    } else {
        digitalWrite(PIN_DRIVE_IN1, LOW);
        digitalWrite(PIN_DRIVE_IN2, LOW);
        analogWrite(PIN_DRIVE_ENA, 0);
    }

    // Right Motor Direction & PWM
    if (right > 0) {
        digitalWrite(PIN_DRIVE_IN3, HIGH);
        digitalWrite(PIN_DRIVE_IN4, LOW);
        analogWrite(PIN_DRIVE_ENB, right);
    } else if (right < 0) {
        digitalWrite(PIN_DRIVE_IN3, LOW);
        digitalWrite(PIN_DRIVE_IN4, HIGH);
        analogWrite(PIN_DRIVE_ENB, -right);
    } else {
        digitalWrite(PIN_DRIVE_IN3, LOW);
        digitalWrite(PIN_DRIVE_IN4, LOW);
        analogWrite(PIN_DRIVE_ENB, 0);
    }
}
