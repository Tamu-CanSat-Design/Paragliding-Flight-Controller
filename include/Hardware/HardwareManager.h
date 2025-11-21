#ifndef HARDWARE_HARDWAREMANAGER_H
#define HARDWARE_HARDWAREMANAGER_H

#include "Common.h"

class HardwareManager{
public:
    void Initialize();
    void ReadSensors();
    void UpdateTelemetry();
    
private:
    Common::telemetry telemetryData; 
    
};

#endif // HARDWARE_HARDWAREMANAGER_H