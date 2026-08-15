#ifndef COMMLINK_H
#define COMMLINK_H

#include "Arduino.h"
#include <cstdio>
#include "Common.h"

class Commlink{
public:
    bool Initialize();
    void WriteGroundRadio(const char* data);
    void ReadGroundRadio(char* buffer, size_t length);
    bool CheckCommand(char* command);

    inline HardwareSerial& GetXbeeSerial() { return Serial7; }
    
private:
    static int index;
};

#endif // COMMLINK_H