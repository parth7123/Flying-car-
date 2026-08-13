#include "RCInput.h"

// Define static member variables
volatile uint16_t RCInput::pulseThrottle = 1000;
volatile uint16_t RCInput::pulsePitch    = 1500;
volatile uint16_t RCInput::pulseRoll     = 1500;
volatile uint16_t RCInput::pulseYaw      = 1500;

volatile uint32_t RCInput::startTimeThrottle = 0;
volatile uint32_t RCInput::startTimePitch    = 0;
volatile uint32_t RCInput::startTimeRoll     = 0;
volatile uint32_t RCInput::startTimeYaw      = 0;

volatile uint8_t  RCInput::lastPinStatePortB = 0;
volatile uint32_t RCInput::lastValidPulseTime = 0;

// Global ISR for Port B Pin Change Interrupts (Pins 10, 11, 12)
ISR(PCINT0_vect) {
    RCInput::handlePCINT0();
}

RCInput::RCInput() : lastPin9State(LOW) {}

void RCInput::begin() {
    pinMode(PIN_RC_THROTTLE, INPUT);
    pinMode(PIN_RC_PITCH, INPUT);
    pinMode(PIN_RC_ROLL, INPUT);
    pinMode(PIN_RC_YAW, INPUT);

    lastValidPulseTime = millis();
    lastPinStatePortB = PINB;

    // Enable Pin Change Interrupt Bank 0 for Pins 10 (PB4), 11 (PB5), 12 (PB6)
    cli();
    PCICR |= (1 << PCIE0);                             // Enable PCINT0 vector
    PCMSK0 |= (1 << PCINT4) | (1 << PCINT5) | (1 << PCINT6); // Mask pins 10, 11, 12
    sei();
}

void RCInput::handlePCINT0() {
    uint32_t now = micros();
    uint8_t currentPinState = PINB;
    uint8_t changedPins = currentPinState ^ lastPinStatePortB;
    lastPinStatePortB = currentPinState;

    // Pin 10 (PB4 - Pitch)
    if (changedPins & (1 << PB4)) {
        if (currentPinState & (1 << PB4)) {
            startTimePitch = now;
        } else {
            uint16_t duration = (uint16_t)(now - startTimePitch);
            if (duration >= 800 && duration <= 2200) {
                pulsePitch = duration;
                lastValidPulseTime = millis();
            }
        }
    }

    // Pin 11 (PB5 - Roll)
    if (changedPins & (1 << PB5)) {
        if (currentPinState & (1 << PB5)) {
            startTimeRoll = now;
        } else {
            uint16_t duration = (uint16_t)(now - startTimeRoll);
            if (duration >= 800 && duration <= 2200) {
                pulseRoll = duration;
                lastValidPulseTime = millis();
            }
        }
    }

    // Pin 12 (PB6 - Yaw)
    if (changedPins & (1 << PB6)) {
        if (currentPinState & (1 << PB6)) {
            startTimeYaw = now;
        } else {
            uint16_t duration = (uint16_t)(now - startTimeYaw);
            if (duration >= 800 && duration <= 2200) {
                pulseYaw = duration;
                lastValidPulseTime = millis();
            }
        }
    }
}

void RCInput::update() {
    // Non-blocking microsecond edge detection for Pin 9 (Throttle)
    uint32_t now = micros();
    uint8_t state9 = digitalRead(PIN_RC_THROTTLE);

    if (state9 != lastPin9State) {
        if (state9 == HIGH) {
            startTimeThrottle = now;
        } else {
            uint16_t duration = (uint16_t)(now - startTimeThrottle);
            if (duration >= 800 && duration <= 2200) {
                pulseThrottle = duration;
                lastValidPulseTime = millis();
            }
        }
        lastPin9State = state9;
    }
}

uint16_t RCInput::getThrottle() const {
    return constrain(pulseThrottle, (uint16_t)RC_MIN_PULSE, (uint16_t)RC_MAX_PULSE);
}

uint16_t RCInput::getPitch() const {
    return constrain(pulsePitch, (uint16_t)RC_MIN_PULSE, (uint16_t)RC_MAX_PULSE);
}

uint16_t RCInput::getRoll() const {
    return constrain(pulseRoll, (uint16_t)RC_MIN_PULSE, (uint16_t)RC_MAX_PULSE);
}

uint16_t RCInput::getYaw() const {
    return constrain(pulseYaw, (uint16_t)RC_MIN_PULSE, (uint16_t)RC_MAX_PULSE);
}

bool RCInput::isFailsafeActive() const {
    return (millis() - lastValidPulseTime) > FAILSAFE_TIMEOUT_MS;
}
