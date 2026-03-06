#ifndef COMMLINK_H
#define COMMLINK_H

#include "Arduino.h"
#include <cstdio>
#include "Common.h"

class Commlink{
public:
    void Initialize();
    void WriteGroundRadio(const char* data);
    void ReadGroundRadio(char* buffer, int length);
    bool CheckCommand(const char* command);

    inline HardwareSerial& GetXbeeSerial() { return Serial1; }
    
private:
    static int index;
};

#endif // COMMLINK_H