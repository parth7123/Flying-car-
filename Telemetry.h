#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include <TinyGPS++.h>
#include "Config.h"
#include "Drive.h"

enum GsmSmsState {
    GSM_IDLE,
    GSM_SET_CMGF,
    GSM_SET_CMGS,
    GSM_SEND_BODY,
    GSM_WAIT_RESPONSE
};

class Telemetry {
public:
    Telemetry();
    void begin();
    void update(Drive &drive);

    // GPS Status Getters
    bool hasGpsFix() const { return gps.location.isValid(); }
    double getLatitude() const { return gps.location.lat(); }
    double getLongitude() const { return gps.location.lng(); }
    double getAltitudeMeters() const { return gps.altitude.meters(); }
    uint32_t getSatellites() const { return gps.satellites.value(); }

    // GSM SMS trigger
    void sendLocationSMS(const char* phoneNumber);

private:
    TinyGPSPlus gps;

    GsmSmsState smsState;
    uint32_t smsTimerMs;
    char smsTargetNumber[20];

    char btBuffer[64];
    uint8_t btBufferIdx;

    void processBluetoothCommand(const char* cmd, Drive &drive);
    void updateGsmStateMachine();
};

#endif // TELEMETRY_H
