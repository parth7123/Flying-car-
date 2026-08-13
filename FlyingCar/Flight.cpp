#include "Flight.h"

Flight::Flight()
    : armState(STATE_DISARMED), armGestureStartMs(0), disarmGestureStartMs(0),
      rollI(0.0f), pitchI(0.0f), yawI(0.0f),
      prevRollError(0.0f), prevPitchError(0.0f), prevYawError(0.0f),
      rollPIDOutput(0.0f), pitchPIDOutput(0.0f), yawPIDOutput(0.0f),
      motorFL(ESC_MIN_PULSE), motorFR(ESC_MIN_PULSE),
      motorRL(ESC_MIN_PULSE), motorRR(ESC_MIN_PULSE),
      inFailsafeRamp(false), failsafeStartMs(0), failsafeInitialThrottle(ESC_MIN_PULSE) {}

void Flight::begin() {
    escFL.attach(PIN_ESC_FL, ESC_MIN_PULSE, ESC_MAX_PULSE);
    escFR.attach(PIN_ESC_FR, ESC_MIN_PULSE, ESC_MAX_PULSE);
    escRL.attach(PIN_ESC_RL, ESC_MIN_PULSE, ESC_MAX_PULSE);
    escRR.attach(PIN_ESC_RR, ESC_MIN_PULSE, ESC_MAX_PULSE);

    disarm();
}

void Flight::disarm() {
    armState = STATE_DISARMED;
    inFailsafeRamp = false;
    rollI = 0.0f;
    pitchI = 0.0f;
    yawI = 0.0f;
    prevRollError = 0.0f;
    prevPitchError = 0.0f;
    prevYawError = 0.0f;
    writeMotors(ESC_MIN_PULSE, ESC_MIN_PULSE, ESC_MIN_PULSE, ESC_MIN_PULSE);
}

void Flight::updateArmingStateMachine(uint16_t throttle, uint16_t yaw) {
    uint32_t now = millis();

    if (armState == STATE_DISARMED) {
        // Arming Gesture: Throttle Low + Yaw Full Right held for ARM_HOLD_TIME_MS
        if (throttle < ARM_STICK_THRESHOLD_LOW && yaw > ARM_STICK_THRESHOLD_HIGH) {
            if (armGestureStartMs == 0) {
                armGestureStartMs = now;
            } else if (now - armGestureStartMs >= ARM_HOLD_TIME_MS) {
                armState = STATE_ARMED;
                armGestureStartMs = 0;
                rollI = 0.0f; pitchI = 0.0f; yawI = 0.0f;
            }
        } else {
            armGestureStartMs = 0;
        }
    } else if (armState == STATE_ARMED) {
        // Disarming Gesture: Throttle Low + Yaw Full Left held for ARM_HOLD_TIME_MS
        if (throttle < ARM_STICK_THRESHOLD_LOW && yaw < ARM_STICK_THRESHOLD_LOW) {
            if (disarmGestureStartMs == 0) {
                disarmGestureStartMs = now;
            } else if (now - disarmGestureStartMs >= ARM_HOLD_TIME_MS) {
                disarm();
                disarmGestureStartMs = 0;
            }
        } else {
            disarmGestureStartMs = 0;
        }
    }
}

void Flight::computePID(uint16_t rcRoll, uint16_t rcPitch, uint16_t rcYaw, const IMU &imu, float dt) {
    if (dt <= 0.0f) return;

    // Convert RC stick values to target setpoints
    float targetRollAngle  = 0.0f;
    float targetPitchAngle = 0.0f;
    float targetYawRate    = 0.0f;

    // Apply deadzones around center 1500us
    if (abs((int16_t)rcRoll - RC_MID_PULSE) > RC_DEADZONE) {
        targetRollAngle = map((long)rcRoll, RC_MIN_PULSE, RC_MAX_PULSE, (long)(-MAX_ROLL_ANGLE_DEG * 10), (long)(MAX_ROLL_ANGLE_DEG * 10)) / 10.0f;
    }
    if (abs((int16_t)rcPitch - RC_MID_PULSE) > RC_DEADZONE) {
        targetPitchAngle = map((long)rcPitch, RC_MIN_PULSE, RC_MAX_PULSE, (long)(-MAX_PITCH_ANGLE_DEG * 10), (long)(MAX_PITCH_ANGLE_DEG * 10)) / 10.0f;
    }
    if (abs((int16_t)rcYaw - RC_MID_PULSE) > RC_DEADZONE) {
        targetYawRate = map((long)rcYaw, RC_MIN_PULSE, RC_MAX_PULSE, (long)(-MAX_YAW_RATE_DEGS * 10), (long)(MAX_YAW_RATE_DEGS * 10)) / 10.0f;
    }

    // Roll Axis PID (Angle setpoint vs estimated angle)
    float rollError = targetRollAngle - imu.getRoll();
    rollI += rollError * dt;
    rollI = constrain(rollI, -ROLL_MAX_I, ROLL_MAX_I);
    float rollD = (dt > 0) ? (rollError - prevRollError) / dt : 0.0f;
    prevRollError = rollError;
    rollPIDOutput = (ROLL_KP * rollError) + (ROLL_KI * rollI) + (ROLL_KD * rollD);

    // Pitch Axis PID (Angle setpoint vs estimated angle)
    float pitchError = targetPitchAngle - imu.getPitch();
    pitchI += pitchError * dt;
    pitchI = constrain(pitchI, -PITCH_MAX_I, PITCH_MAX_I);
    float pitchD = (dt > 0) ? (pitchError - prevPitchError) / dt : 0.0f;
    prevPitchError = pitchError;
    pitchPIDOutput = (PITCH_KP * pitchError) + (PITCH_KI * pitchI) + (PITCH_KD * pitchD);

    // Yaw Axis PID (Rate setpoint vs gyro rate)
    float yawError = targetYawRate - imu.getGyroZ();
    yawI += yawError * dt;
    yawI = constrain(yawI, -YAW_MAX_I, YAW_MAX_I);
    float yawD = (dt > 0) ? (yawError - prevYawError) / dt : 0.0f;
    prevYawError = yawError;
    yawPIDOutput = (YAW_KP * yawError) + (YAW_KI * yawI) + (YAW_KD * yawD);
}

void Flight::update(const RCInput &rc, const IMU &imu, float dt) {
    uint16_t throttle = rc.getThrottle();
    uint16_t pitch    = rc.getPitch();
    uint16_t roll     = rc.getRoll();
    uint16_t yaw      = rc.getYaw();

    // Check RC Failsafe logic
    if (rc.isFailsafeActive()) {
        if (armState == STATE_ARMED) {
            uint32_t now = millis();
            if (!inFailsafeRamp) {
                inFailsafeRamp = true;
                failsafeStartMs = now;
                failsafeInitialThrottle = throttle;
            }

            uint32_t elapsed = now - failsafeStartMs;
            if (elapsed >= FAILSAFE_RAMP_TIME_MS) {
                disarm();
                return;
            } else {
                // Ramp throttle linearly down to minimum
                float progress = (float)elapsed / (float)FAILSAFE_RAMP_TIME_MS;
                throttle = (uint16_t)(failsafeInitialThrottle * (1.0f - progress) + ESC_MIN_PULSE * progress);
            }
        } else {
            disarm();
            return;
        }
    } else {
        inFailsafeRamp = false;
        // Update arming state machine
        updateArmingStateMachine(throttle, yaw);
    }

    if (armState != STATE_ARMED) {
        writeMotors(ESC_MIN_PULSE, ESC_MIN_PULSE, ESC_MIN_PULSE, ESC_MIN_PULSE);
        return;
    }

    // Don't accumulate PID if throttle is at ground idle
    if (throttle < ESC_IDLE_PULSE) {
        rollI = 0.0f;
        pitchI = 0.0f;
        yawI = 0.0f;
        writeMotors(ESC_MIN_PULSE, ESC_MIN_PULSE, ESC_MIN_PULSE, ESC_MIN_PULSE);
        return;
    }

    // Compute PID stabilization
    computePID(roll, pitch, yaw, imu, dt);

    // Quad-X Motor Mixing Matrix
    // FL = throttle + pitch - roll + yaw
    // FR = throttle + pitch + roll - yaw
    // RL = throttle - pitch - roll - yaw
    // RR = throttle - pitch + roll + yaw
    int32_t fl = (int32_t)throttle + (int32_t)pitchPIDOutput - (int32_t)rollPIDOutput + (int32_t)yawPIDOutput;
    int32_t fr = (int32_t)throttle + (int32_t)pitchPIDOutput + (int32_t)rollPIDOutput - (int32_t)yawPIDOutput;
    int32_t rl = (int32_t)throttle - (int32_t)pitchPIDOutput - (int32_t)rollPIDOutput - (int32_t)yawPIDOutput;
    int32_t rr = (int32_t)throttle - (int32_t)pitchPIDOutput + (int32_t)rollPIDOutput + (int32_t)yawPIDOutput;

    // Strict safety bounds checking [1000 - 2000] us
    uint16_t outFL = (uint16_t)constrain(fl, ESC_MIN_PULSE, ESC_MAX_PULSE);
    uint16_t outFR = (uint16_t)constrain(fr, ESC_MIN_PULSE, ESC_MAX_PULSE);
    uint16_t outRL = (uint16_t)constrain(rl, ESC_MIN_PULSE, ESC_MAX_PULSE);
    uint16_t outRR = (uint16_t)constrain(rr, ESC_MIN_PULSE, ESC_MAX_PULSE);

    writeMotors(outFL, outFR, outRL, outRR);
}

void Flight::writeMotors(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr) {
    motorFL = fl;
    motorFR = fr;
    motorRL = rl;
    motorRR = rr;

    escFL.writeMicroseconds(motorFL);
    escFR.writeMicroseconds(motorFR);
    escRL.writeMicroseconds(motorRL);
    escRR.writeMicroseconds(motorRR);
}
