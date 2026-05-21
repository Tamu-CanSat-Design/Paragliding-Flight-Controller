#include "Commlink.h"

int Commlink::index = 0;

bool Commlink::Initialize(){
/* Initialize Ground Link */

    GetXbeeSerial().begin(Common::XBEE_BAUDRATE);
    return true;
}


void Commlink::WriteGroundRadio(const char* data){
/* If data exists, write to XBEE and flush buffer */
    if (!data){
        return;
    }
    GetXbeeSerial().println(data);
    GetXbeeSerial().flush();
   
}
void Commlink::ReadGroundRadio(char* buffer, size_t length){
/* If data exists, parse */
    if (!buffer || length <= 1) {
        return;
    }

    while (GetXbeeSerial().available() > 0) {
        int c = GetXbeeSerial().read();
        // ignore CR characters (common on serial line endings \r\n)
        if (c == '\r') continue;
        if (c == '\n') {
            buffer[index] = '\0';
            index = 0;
        } else {
            if ((size_t)index < length - 1) {
                buffer[index++] = static_cast<char>(c);
            } else {
                index = 0;
                return;
            }
        }
    }
}

// copy next field into dst
static void copyField(const char*& s, char* dst, size_t dstSize) {
    // skip leading whitespace and CR that may remain from serial input
    while (*s == ' ' || *s == '\t' || *s == '\r') ++s;
    size_t i = 0;
    while (*s && *s != ',' && i + 1 < dstSize) {
        dst[i++] = *s++;
    }
    dst[i] = '\0';
    if (*s == ',' || *s == '\r') ++s;
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

