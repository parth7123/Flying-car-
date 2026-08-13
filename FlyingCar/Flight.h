#ifndef FLIGHT_H
#define FLIGHT_H

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"
#include "IMU.h"
#include "RCInput.h"

enum ArmState {
    STATE_DISARMED,
    STATE_ARMING,
    STATE_ARMED
};

class Flight {
public:
    Flight();
    void begin();
    void update(const RCInput &rc, const IMU &imu, float dt);
    void disarm();

    bool isArmed() const { return armState == STATE_ARMED; }
    ArmState getArmState() const { return armState; }

    uint16_t getMotorFL() const { return motorFL; }
    uint16_t getMotorFR() const { return motorFR; }
    uint16_t getMotorRL() const { return motorRL; }
    uint16_t getMotorRR() const { return motorRR; }

    float getRollPIDOutput() const { return rollPIDOutput; }
    float getPitchPIDOutput() const { return pitchPIDOutput; }
    float getYawPIDOutput() const { return yawPIDOutput; }

private:
    Servo escFL;
    Servo escFR;
    Servo escRL;
    Servo escRR;

    ArmState armState;
    uint32_t armGestureStartMs;
    uint32_t disarmGestureStartMs;

    // PID states
    float rollI, pitchI, yawI;
    float prevRollError, prevPitchError, prevYawError;

    float rollPIDOutput;
    float pitchPIDOutput;
    float yawPIDOutput;

    uint16_t motorFL;
    uint16_t motorFR;
    uint16_t motorRL;
    uint16_t motorRR;

    // Failsafe handling
    bool inFailsafeRamp;
    uint32_t failsafeStartMs;
    uint16_t failsafeInitialThrottle;

    void updateArmingStateMachine(uint16_t throttle, uint16_t yaw);
    void computePID(uint16_t rcRoll, uint16_t rcPitch, uint16_t rcYaw, const IMU &imu, float dt);
    void writeMotors(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr);
};

#endif // FLIGHT_H
