#ifndef COMMLINK_H
#define COMMLINK_H
#define XBEE_SERIAL   Serial1
#define XBEE_BAUDRATE 115200
#include "Arduino.h"
#include <string.h>
#include <cstdio>
#include "Common.h"

class Commlink{
public:
    void Initialize();
    void WriteGroundRadio(const char* data);
    void ReadGroundRadio(char* buffer, int length);
    void ParseCommand(const char* command);
private:
    static int index;
};

#endif // COMMLINK_H