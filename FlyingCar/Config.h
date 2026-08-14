#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// SYSTEM DEBUG CONFIGURATION
// ==========================================
#define DEBUG                 1    // Set to 1 to enable Serial debugging, 0 to disable
#define DEBUG_BAUD            115200

// ==========================================
// HARDWARE PIN MAPPINGS (Arduino Mega 2560)
// ==========================================
// Mode Switch
#define PIN_MODE_SWITCH       8    // LOW = Drive Mode, HIGH = Fly Mode

// MPU6050 I2C (Hardware Wire pins on Mega 2560)
#define PIN_MPU_SDA           20   // I2C Data
#define PIN_MPU_SCL           21   // I2C Clock

// Flight Motors / ESCs (Servo PWM Output)
#define PIN_ESC_FL            2    // Front-Left Motor (ESC 1)
#define PIN_ESC_FR            3    // Front-Right Motor (ESC 2)
#define PIN_ESC_RL            4    // Rear-Left Motor (ESC 3)
#define PIN_ESC_RR            5    // Rear-Right Motor (ESC 4)

// Ground Drive Motors (L298N H-Bridge Driver)
#define PIN_DRIVE_IN1         30   // Left Motor Direction 1
#define PIN_DRIVE_IN2         31   // Left Motor Direction 2
#define PIN_DRIVE_IN3         32   // Right Motor Direction 1
#define PIN_DRIVE_IN4         33   // Right Motor Direction 2
#define PIN_DRIVE_ENA         6    // Left Motor Speed (PWM)
#define PIN_DRIVE_ENB         7    // Right Motor Speed (PWM)

// RC Receiver Input Channels (PWM Pulses)
#define PIN_RC_THROTTLE       9    // RC Channel 1: Throttle
#define PIN_RC_PITCH          10   // RC Channel 2: Pitch
#define PIN_RC_ROLL           11   // RC Channel 3: Roll
#define PIN_RC_YAW            12   // RC Channel 4: Yaw

// Serial Port Mapping
// Serial1: HC-05 Bluetooth (TX1=18, RX1=19)
// Serial2: SIM800L GSM (TX2=16, RX2=17)
// Serial3: NEO-6M GPS (TX3=14, RX3=15)
#define SERIAL_BT             Serial1
#define SERIAL_GSM            Serial2
#define SERIAL_GPS            Serial3

#define BAUD_BT               9600
#define BAUD_GSM              9600
#define BAUD_GPS              9600

// ==========================================
// SAFETY & ESC BOUNDS
// ==========================================
#define ESC_MIN_PULSE         1000 // Minimum ESC PWM pulse (microseconds)
#define ESC_MAX_PULSE         2000 // Maximum ESC PWM pulse (microseconds)
#define ESC_IDLE_PULSE        1100 // Arming idle throttle PWM (microseconds)

#define RC_MIN_PULSE          1000 // Standard RC minimum pulse width (us)
#define RC_MAX_PULSE          2000 // Standard RC maximum pulse width (us)
#define RC_MID_PULSE          1500 // Standard RC neutral pulse width (us)
#define RC_DEADZONE           15   // Stick neutral deadzone threshold (us)

// Failsafe configuration
#define FAILSAFE_TIMEOUT_MS   500  // RC signal loss timeout trigger (ms)
#define FAILSAFE_RAMP_TIME_MS 1000 // Duration to ramp down throttle on signal loss (ms)

// Arming gesture parameters
#define ARM_STICK_THRESHOLD_LOW  1080 // us
#define ARM_STICK_THRESHOLD_HIGH 1900 // us
#define ARM_HOLD_TIME_MS         1000 // Time required to hold arm stick position (ms)

// Ground drive ramping limit
#define DRIVE_RAMP_STEP       8    // Maximum PWM change per 20ms drive tick (prevents high current spikes)

// ==========================================
// FLIGHT PID & CONTROL TUNING
// ==========================================
#define FLIGHT_LOOP_FREQ_HZ   250  // Target loop frequency (250Hz = 4ms period)
#define FLIGHT_LOOP_TIME_US   4000 // Loop execution target period in microseconds

// Angle limits
#define MAX_ROLL_ANGLE_DEG    30.0f // Maximum tilt angle target (degrees)
#define MAX_PITCH_ANGLE_DEG   30.0f
#define MAX_YAW_RATE_DEGS     180.0f // Maximum target yaw rate (deg/s)

// Default PID Gains
// Roll Axis
#define ROLL_KP               1.20f
#define ROLL_KI               0.02f
#define ROLL_KD               0.35f
#define ROLL_MAX_I            100.0f

// Pitch Axis
#define PITCH_KP              1.20f
#define PITCH_KI              0.02f
#define PITCH_KD              0.35f
#define PITCH_MAX_I           100.0f

// Yaw Axis
#define YAW_KP                2.00f
#define YAW_KI                0.00f
#define YAW_KD                0.10f
#define YAW_MAX_I             50.0f

#endif // CONFIG_H
