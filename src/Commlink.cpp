#include "Commlink.h"

#include <cstring>

int Commlink::index = 0;

namespace {
char gRxLineBuffer[Common::COMMAND_BUFFER_LENGTH] = {};
}

void Commlink::Initialize(){
/* Initialize Ground Link */

    GetXbeeSerial().begin(Common::XBEE_BAUDRATE);
}


void Commlink::WriteGroundRadio(const char* data){
/* If data exists, write to XBEE and flush buffer */
    if (!data){
        return;
    }
    GetXbeeSerial().println(data);
    GetXbeeSerial().flush();
   
}
void Commlink::ReadGroundRadio(char* buffer, int length){
/* If data exists, parse */
    if (!buffer || length <= 1) {
        return;
    }

    buffer[0] = '\0';

    while (GetXbeeSerial().available() > 0) {
        char c = GetXbeeSerial().read();
        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            gRxLineBuffer[index] = '\0';
            std::strncpy(buffer, gRxLineBuffer, length - 1);
            buffer[length - 1] = '\0';
            index = 0;
            gRxLineBuffer[0] = '\0';
            return;
        } else {
            if (index < length - 1) {
                gRxLineBuffer[index++] = c;
            } else {
                index = 0;
                gRxLineBuffer[0] = '\0';
                return;
            }
        }
    }
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

bool Commlink::CheckCommand(const char* command){
    if (!command || !*command) return false;

    const char* p = command;

    // 1) prefix: CMD
    char prefix[4];
    copyField(p, prefix, sizeof(prefix));
    if (strcmp(prefix, "CMD") != 0) {
        Serial.println("Invalid prefix");
        return false;
    }

    // 2) prefix: TEAM_ID
    char teamId[8];
    copyField(p, teamId, sizeof(teamId));
    if (atoi(teamId) != Common::TEAM_ID) {
        Serial.println("Invalid TEAM_ID");
        return false;
    }
    return true;
}
