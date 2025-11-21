#ifndef HARDWARE_HARDWAREMANAGER_H
#define HARDWARE_HARDWAREMANAGER_H
#include "EEPROM.h"
#include "Common.h"
#include "BMP390.cpp" //Barometer used
//#include "GPS"
//#include "IMU"
//#include "VoltageSensor"
//#include "CurrentSensor"
//#include "Magnetometer"
#include "HealthMonitor.h"
class HardwareManager{
public:
    void Initialize();
    void ReadSensors();
    void UpdateTelemetry();
    
private:
    Common::telemetry _telemetryData; 
    BMP390 _barometer;
};

#endif // HARDWARE_HARDWAREMANAGER_H