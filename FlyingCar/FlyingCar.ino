/*
  ===================================================================================
  HYBRID FLYING RC CAR FIRMWARE — ARDUINO MEGA 2560
  ===================================================================================
  A hybrid ground vehicle and quadcopter flight controller firmware featuring dual-mode
  operation, MPU6050 complementary filter flight stabilization, Quad-X ESC mixing,
  L298N skid steering with gearbox protection ramping, PCINT interrupt RC pulse decoding,
  RC signal loss failsafes, and non-blocking GPS / Bluetooth / GSM telemetry.

  WIRING ASSUMPTIONS & PIN MAP:
  -----------------------------------------------------------------------------------
  Function                          Arduino Mega Pin
  -----------------------------------------------------------------------------------
  MPU6050 SDA (I2C)                 Pin 20
  MPU6050 SCL (I2C)                 Pin 21
  ESC 1 (Front-Left Motor)          Pin 2 (Servo PWM 1000-2000us)
  ESC 2 (Front-Right Motor)         Pin 3 (Servo PWM 1000-2000us)
  ESC 3 (Rear-Left Motor)           Pin 4 (Servo PWM 1000-2000us)
  ESC 4 (Rear-Right Motor)          Pin 5 (Servo PWM 1000-2000us)
  DC Motor Driver IN1 / IN2 (Left)  Pin 22, Pin 23
  DC Motor Driver IN3 / IN4 (Right) Pin 24, Pin 25
  DC Motor Driver ENA (Left PWM)    Pin 6
  DC Motor Driver ENB (Right PWM)   Pin 7
  Mode Switch                       Pin 8 (LOW = Drive Mode, HIGH = Fly Mode)
  RC Receiver Throttle (Ch 1)       Pin 9
  RC Receiver Pitch    (Ch 2)       Pin 10 (PCINT4)
  RC Receiver Roll     (Ch 3)       Pin 11 (PCINT5)
  RC Receiver Yaw      (Ch 4)       Pin 12 (PCINT6)
  HC-05 Bluetooth                   Serial1 (TX1=18, RX1=19)
  SIM800L GSM Telemetry             Serial2 (TX2=16, RX2=17)
  NEO-6M GPS Module                 Serial3 (TX3=14, RX3=15)
  ===================================================================================
*/

#include <Arduino.h>
#include "Config.h"
#include "IMU.h"
#include "RCInput.h"
#include "Drive.h"
#include "Flight.h"
#include "Telemetry.h"

enum VehicleMode {
    MODE_DRIVE,
    MODE_FLY,
    MODE_TRANSITION
};

// Global Subsystem Objects
IMU       imu;
RCInput   rc;
Drive     drive;
Flight    flight;
Telemetry telemetry;

VehicleMode currentMode   = MODE_DRIVE;
VehicleMode targetMode    = MODE_DRIVE;
uint32_t transitionStartMs = 0;

uint32_t lastFlightLoopUs  = 0;
uint32_t lastDebugPrintMs  = 0;

void printDebugInfo();

void setup() {
#if DEBUG
    Serial.begin(DEBUG_BAUD);
    delay(500); // Allow USB Serial port on Mega 2560 to stabilize after reset
    Serial.println();
    Serial.println(F("=================================================="));
    Serial.println(F(" INITIALIZING HYBRID FLYING RC CAR FIRMWARE"));
    Serial.println(F(" Board: Arduino Mega 2560 | Target loop: 250Hz"));
    Serial.println(F("=================================================="));
    Serial.flush();
#endif

    // Initialize Mode Switch Pin (Internal Pull-Up enabled)
    pinMode(PIN_MODE_SWITCH, INPUT_PULLUP);

    // Initialize Subsystems
    drive.begin();
    flight.begin();
    rc.begin();
    telemetry.begin();

#if DEBUG
    Serial.println(F("[IMU] Initializing MPU6050..."));
#endif
    if (!imu.begin()) {
#if DEBUG
        Serial.println(F("[ERROR] MPU6050 Initialization Failed! Check I2C Wiring."));
#endif
    } else {
#if DEBUG
        Serial.println(F("[IMU] MPU6050 Initialized & Calibrated Successfully."));
#endif
    }

    // Determine initial operating mode
    bool rawModeState = digitalRead(PIN_MODE_SWITCH);
    currentMode = rawModeState ? MODE_FLY : MODE_DRIVE;
    targetMode  = currentMode;

#if DEBUG
    Serial.print(F("[SYSTEM] Initial Mode: "));
    Serial.println(currentMode == MODE_FLY ? F("FLY MODE") : F("DRIVE MODE"));
    Serial.println(F("[SYSTEM] Initialization Complete. Entering Main Loop."));
#endif

    lastFlightLoopUs = micros();
}

void loop() {
    uint32_t nowUs = micros();
    uint32_t nowMs = millis();

    // 1. Service non-blocking RC pulse decoding and telemetry streams
    rc.update();
    telemetry.update(drive);

    // 2. Read Mode Switch & handle mode transitions
    bool rawSwitchState = digitalRead(PIN_MODE_SWITCH);
    VehicleMode readMode = rawSwitchState ? MODE_FLY : MODE_DRIVE;

    if (readMode != targetMode) {
        // Mode switch toggled mid-operation: enforce immediate safe motor lockout
        targetMode = readMode;
        currentMode = MODE_TRANSITION;
        transitionStartMs = nowMs;
        drive.stop();
        flight.disarm();
#if DEBUG
        Serial.println(F("\n[MODE] *** MODE SWITCH DETECTED *** Safe stop forced. Initiating transition..."));
#endif
    }

    if (currentMode == MODE_TRANSITION) {
        drive.stop();
        flight.disarm();
        if (nowMs - transitionStartMs >= 200) { // 200ms safety lockout guard
            currentMode = targetMode;
#if DEBUG
            Serial.print(F("[MODE] Transition Complete. Active Mode: "));
            Serial.println(currentMode == MODE_FLY ? F("FLY MODE") : F("DRIVE MODE"));
#endif
        }
    }

    // 3. High-Speed Flight Stabilization Loop (250 Hz = 4000us period)
    if (nowUs - lastFlightLoopUs >= FLIGHT_LOOP_TIME_US) {
        float dt = (nowUs - lastFlightLoopUs) / 1000000.0f;
        lastFlightLoopUs = nowUs;

        // Update MPU6050 complementary filter estimation
        imu.update(dt);

        if (currentMode == MODE_FLY) {
            // Drive ground motors strictly stopped in Fly Mode
            drive.stop();

            // Run Flight Controller PID loops and ESC write
            flight.update(rc, imu, dt);
        } else if (currentMode == MODE_DRIVE) {
            // ESCs disarmed strictly in Drive Mode
            flight.disarm();

            // Allow RC Transmitter sticks to drive tires directly if active
            if (!rc.isFailsafeActive()) {
                int16_t rcForward = 0;
                int16_t rcSteer = 0;
                uint16_t pit = rc.getPitch();
                uint16_t rol = rc.getRoll();
                if (abs((int16_t)pit - RC_MID_PULSE) > RC_DEADZONE) {
                    rcForward = map((long)pit, RC_MIN_PULSE, RC_MAX_PULSE, -255, 255);
                }
                if (abs((int16_t)rol - RC_MID_PULSE) > RC_DEADZONE) {
                    rcSteer = map((long)rol, RC_MIN_PULSE, RC_MAX_PULSE, -255, 255);
                }
                if (rcForward != 0 || rcSteer != 0) {
                    drive.setCommand(rcForward, rcSteer);
                }
            }

            // Run Ground Drive Ramping Loop
            drive.update();
        }
    }

    // 4. Debugging & Telemetry Output (Toggleable via DEBUG flag in Config.h)
#if DEBUG
    if (nowMs - lastDebugPrintMs >= 250) { // 4 Hz debug output rate
        lastDebugPrintMs = nowMs;
        printDebugInfo();
    }
#endif
}

#if DEBUG
void printDebugInfo() {
    Serial.print(F("MODE: "));
    if (currentMode == MODE_DRIVE)       Serial.print(F("DRIVE"));
    else if (currentMode == MODE_FLY)    Serial.print(F("FLY  "));
    else                                 Serial.print(F("TRANS"));

    Serial.print(F(" | ARM: "));
    Serial.print(flight.isArmed() ? F("ARMED   ") : F("DISARMED"));

    Serial.print(F(" | RC Thr: ")); Serial.print(rc.getThrottle());
    Serial.print(F(" Pit: "));       Serial.print(rc.getPitch());
    Serial.print(F(" Rol: "));       Serial.print(rc.getRoll());
    Serial.print(F(" Yaw: "));       Serial.print(rc.getYaw());

    if (rc.isFailsafeActive()) {
        Serial.print(F(" [FAILSAFE]"));
    }

    if (currentMode == MODE_FLY) {
        Serial.print(F(" | IMU Roll: "));  Serial.print(imu.getRoll(), 1);
        Serial.print(F(" Pit: "));          Serial.print(imu.getPitch(), 1);
        Serial.print(F(" | ESC FL: "));     Serial.print(flight.getMotorFL());
        Serial.print(F(" FR: "));           Serial.print(flight.getMotorFR());
        Serial.print(F(" RL: "));           Serial.print(flight.getMotorRL());
        Serial.print(F(" RR: "));           Serial.print(flight.getMotorRR());
    } else {
        Serial.print(F(" | Motor Left: "));  Serial.print(drive.getCurrentLeftSpeed());
        Serial.print(F(" Right: "));         Serial.print(drive.getCurrentRightSpeed());
    }

    Serial.print(F(" | GPS: "));
    if (telemetry.hasGpsFix()) {
        Serial.print(F("FIX (Lat: "));
        Serial.print(telemetry.getLatitude(), 5);
        Serial.print(F(" Lng: "));
        Serial.print(telemetry.getLongitude(), 5);
        Serial.print(F(" Sats: "));
        Serial.print(telemetry.getSatellites());
        Serial.print(F(")"));
    } else {
        Serial.print(F("NO FIX"));
    }

    Serial.println();
}
#endif
