#include "Telemetry.h"
#include <stdio.h>

Telemetry::Telemetry() 
    : smsState(GSM_IDLE), smsTimerMs(0), btBufferIdx(0) {
    smsTargetNumber[0] = '\0';
    btBuffer[0] = '\0';
}

void Telemetry::begin() {
    SERIAL_BT.begin(BAUD_BT);
    SERIAL_GSM.begin(BAUD_GSM);
    SERIAL_GPS.begin(BAUD_GPS);

    // Basic GSM initialization check
    SERIAL_GSM.println("AT");
}

void Telemetry::sendLocationSMS(const char* phoneNumber) {
    if (smsState != GSM_IDLE) return; // Busy sending previous SMS

    if (phoneNumber && strlen(phoneNumber) > 0) {
        strncpy(smsTargetNumber, phoneNumber, sizeof(smsTargetNumber) - 1);
        smsTargetNumber[sizeof(smsTargetNumber) - 1] = '\0';
    } else {
        strcpy(smsTargetNumber, "+1234567890"); // Default placeholder if none provided
    }

    smsState = GSM_SET_CMGF;
    smsTimerMs = millis();
}

void Telemetry::processBluetoothCommand(const char* cmd, Drive &drive) {
    if (!cmd || strlen(cmd) == 0) return;

    // Check for drive command format: "D,<forward>,<steer>"
    if (cmd[0] == 'D' && cmd[1] == ',') {
        int forward = 0;
        int steer = 0;
        if (sscanf(cmd + 2, "%d,%d", &forward, &steer) == 2) {
            drive.setCommand(forward, steer);
        }
        return;
    }

    // Check single key directional commands
    switch (cmd[0]) {
        case 'F': // Forward full speed
            drive.setCommand(200, 0);
            break;
        case 'B': // Backward
            drive.setCommand(-200, 0);
            break;
        case 'L': // Turn Left
            drive.setCommand(0, -180);
            break;
        case 'R': // Turn Right
            drive.setCommand(0, 180);
            break;
        case 'S': // Stop
            drive.setCommand(0, 0);
            break;
        default:
            break;
    }

    // Check Location SMS command: "LOC" or "LOC,<phoneNumber>"
    if (strncmp(cmd, "LOC", 3) == 0) {
        if (cmd[3] == ',' && strlen(cmd) > 4) {
            sendLocationSMS(cmd + 4);
        } else {
            sendLocationSMS("+1234567890");
        }
    }
}

void Telemetry::updateGsmStateMachine() {
    uint32_t now = millis();

    switch (smsState) {
        case GSM_IDLE:
            break;

        case GSM_SET_CMGF:
            SERIAL_GSM.println("AT+CMGF=1"); // Set SMS text mode
            smsTimerMs = now;
            smsState = GSM_SET_CMGS;
            break;

        case GSM_SET_CMGS:
            if (now - smsTimerMs >= 300) {
                SERIAL_GSM.print("AT+CMGS=\"");
                SERIAL_GSM.print(smsTargetNumber);
                SERIAL_GSM.println("\"");
                smsTimerMs = now;
                smsState = GSM_SEND_BODY;
            }
            break;

        case GSM_SEND_BODY:
            if (now - smsTimerMs >= 500) {
                SERIAL_GSM.print("Vehicle Location:\n");
                if (hasGpsFix()) {
                    SERIAL_GSM.print("Lat: ");
                    SERIAL_GSM.print(getLatitude(), 6);
                    SERIAL_GSM.print("\nLng: ");
                    SERIAL_GSM.print(getLongitude(), 6);
                    SERIAL_GSM.print("\nGoogle Maps: https://maps.google.com/?q=");
                    SERIAL_GSM.print(getLatitude(), 6);
                    SERIAL_GSM.print(",");
                    SERIAL_GSM.print(getLongitude(), 6);
                } else {
                    SERIAL_GSM.print("GPS Fix: NO FIX YET");
                }
                SERIAL_GSM.write(0x1A); // Send Ctrl+Z to submit SMS
                smsTimerMs = now;
                smsState = GSM_WAIT_RESPONSE;
            }
            break;

        case GSM_WAIT_RESPONSE:
            if (now - smsTimerMs >= 3000) {
                smsState = GSM_IDLE; // Completed SMS transmit task
            }
            break;
    }

    // Drain incoming serial buffer from GSM module to prevent overflow
    while (SERIAL_GSM.available()) {
        SERIAL_GSM.read();
    }
}

void Telemetry::update(Drive &drive) {
    // 1. Parse NEO-6M GPS data streams
    while (SERIAL_GPS.available()) {
        char c = SERIAL_GPS.read();
        gps.encode(c);
    }

    // 2. Parse HC-05 Bluetooth command stream
    while (SERIAL_BT.available()) {
        char c = SERIAL_BT.read();
        if (c == '\n' || c == '\r') {
            if (btBufferIdx > 0) {
                btBuffer[btBufferIdx] = '\0';
                processBluetoothCommand(btBuffer, drive);
                btBufferIdx = 0;
            }
        } else if (btBufferIdx < sizeof(btBuffer) - 1) {
            btBuffer[btBufferIdx++] = c;
        }
    }

    // 3. Service non-blocking GSM SMS transmission state machine
    updateGsmStateMachine();
}
