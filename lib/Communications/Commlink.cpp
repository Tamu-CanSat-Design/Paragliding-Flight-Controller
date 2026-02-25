#include "Commlink.h"
#include "HardwareManager.h"
#include <Arduino.h>
#include <string.h>

int Commlink::index = 0;

void Commlink::Initialize(){
/* Initialize Ground Link */
    XBEE_SERIAL.begin(XBEE_BAUDRATE);
    hardwareManager_ = nullptr;
}

void Commlink::SetHardwareManager(HardwareManager* hwManager){
    hardwareManager_ = hwManager;
}

void Commlink::WriteGroundRadio(const char* data){
/* If data exists, write to XBEE and flush buffer */
    if (!data){
        return;
    }
    XBEE_SERIAL.println(data);
    XBEE_SERIAL.flush();
}
void Commlink::ReadGroundRadio(char* buffer, int length){
/* If data exists, parse */
    if (!buffer || length <= 1) {
        return;
    }

    while (XBEE_SERIAL.available() > 0) {
        char c = XBEE_SERIAL.read();
        if (c == '\n') {
            buffer[index] = '\0';
            ParseCommand(buffer);
            index = 0;
        } else {
            if (index < length - 1) {
                buffer[index++] = c;
            } else {
                index = 0;
                return;
            }
        }
    }
}

// skip one comma‑separated field
static void skipField(const char*& s) {
    while (*s && *s != ',') ++s;
    if (*s == ',') ++s;
}

// copy next field into dst
static void copyField(const char*& s, char* dst, size_t dstSize) {
    size_t i = 0;
    while (*s && *s != ',' && i + 1 < dstSize) {
        dst[i++] = *s++;
    }
    dst[i] = '\0';
    if (*s == ',') ++s;
}

void Commlink::ParseCommand(const char* command){
    if (!command || !*command) return;

    const char* p = command;

    // 1) prefix: CMD
    char prefix[4];
    copyField(p, prefix, sizeof(prefix));
    if (strcmp(prefix, "CMD") != 0) {
        Serial.println("Invalid prefix");
        return;
    }

    // 2) TEAM_ID (ignored here)
    skipField(p);

    // 3) main command: CX, ST, SIM, SIMP, CAL, MEC, ...
    char cmd[8];
    copyField(p, cmd, sizeof(cmd));

    // ---------- CX: Payload Telemetry On/Off ----------
    if (strcmp(cmd, "CX") == 0) {
        char onOff[8];
        copyField(p, onOff, sizeof(onOff));

        if (strcmp(onOff, "ON") == 0) {
            Serial.println("Telemetry ON");
            // TODO: Enable telemetry transmission
        } else if (strcmp(onOff, "OFF") == 0) {
            Serial.println("Telemetry OFF");
            // TODO: Disable telemetry transmission
        } else {
            Serial.println("CX: invalid ON/OFF");
        }
    }

    // ---------- ST: Set Time ----------
    else if (strcmp(cmd, "ST") == 0) {
        char utcTime[24];              // e.g. "15:31:00"
        copyField(p, utcTime, sizeof(utcTime));

        if (*utcTime) {
            Serial.print("Set RTC to UTC ");
            Serial.println(utcTime);
            // TODO: Set RTC time from ground station
        } else {
            Serial.println("ST: missing time");
        }
    }

    // ---------- SIM: Simulation Mode Control ----------
    else if (strcmp(cmd, "SIM") == 0) {
        char mode[16];                 // e.g. "ENABLE" / "DISABLE"
        copyField(p, mode, sizeof(mode));

        if (strcmp(mode, "ENABLE") == 0) {
            Serial.println("Simulation mode ENABLE");
        } else if (strcmp(mode, "DISABLE") == 0) {
            Serial.println("Simulation mode DISABLE");
        } else {
            Serial.println("SIM: unknown mode");
        }
    }

    // ---------- SIMP: Simulated Pressure Data ----------
    else if (strcmp(cmd, "SIMP") == 0) {
        char pressure[16];
        copyField(p, pressure, sizeof(pressure));

        if (*pressure) {
            Serial.print("Simulated pressure = ");
            Serial.println(pressure);
        } else {
            Serial.println("SIMP: missing pressure");
        }
    }

    // ---------- CAL: Calibrate Altitude to Zero ----------
    else if (strcmp(cmd, "CAL") == 0) {
        Serial.println("Calibrate altitude to 0");
    }

    // ---------- MEC: mechanical sub‑commands ----------
    else if (strcmp(cmd, "MEC") == 0) {
        char subcmd[8];
        copyField(p, subcmd, sizeof(subcmd));

        // CPREL: Release Container/Payload
        if (strcmp(subcmd, "CPREL") == 0) {
            char onOff[8];
            copyField(p, onOff, sizeof(onOff));

            if (strcmp(onOff, "ON") == 0) {
                Serial.println("Container/Payload release ACTIVATED");
            } else {
                Serial.println("CPREL: invalid ON/OFF");
            }
        }
        // EGREL: Release Egg Instrument
        else if (strcmp(subcmd, "EGREL") == 0) {
            char onOff[8];
            copyField(p, onOff, sizeof(onOff));

            if (strcmp(onOff, "ON") == 0) {
                Serial.println("Egg instrument release ACTIVATED");
            } else {
                Serial.println("EGREL: invalid ON/OFF");
            }
        }
        // LDAR: Extend LiDAR/Camera
        else if (strcmp(subcmd, "LDAR") == 0) {
            char onOff[8];
            copyField(p, onOff, sizeof(onOff));

            if (strcmp(onOff, "ON") == 0) {
                Serial.println("LiDAR/Camera EXTENDED");
            } else {
                Serial.println("LDAR: invalid ON/OFF");
            }
        }
        else {
            Serial.println("MEC: unknown subcommand");
        }
    }

    // ---------- Unknown main command ----------
    else {
        Serial.print("Unknown command: ");
        Serial.println(cmd);
    }