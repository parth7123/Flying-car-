#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <Arduino.h>
#include "Config.h"

class RCInput {
public:
    RCInput();
    void begin();
    void update();

    // Pulse width getters (microseconds: 1000 - 2000)
    uint16_t getThrottle() const;
    uint16_t getPitch() const;
    uint16_t getRoll() const;
    uint16_t getYaw() const;

    // Failsafe monitoring
    bool isFailsafeActive() const;
    uint32_t getLastValidTime() const { return lastValidPulseTime; }

    // Internal ISR update handlers
    static void handlePCINT0();

private:
    static volatile uint16_t pulseThrottle;
    static volatile uint16_t pulsePitch;
    static volatile uint16_t pulseRoll;
    static volatile uint16_t pulseYaw;

    static volatile uint32_t startTimeThrottle;
    static volatile uint32_t startTimePitch;
    static volatile uint32_t startTimeRoll;
    static volatile uint32_t startTimeYaw;

    static volatile uint8_t lastPinStatePortB;
    static volatile uint32_t lastValidPulseTime;

    // Non-blocking polling for Pin 9 (PH6)
    uint8_t lastPin9State;
};

#endif // RC_INPUT_H
