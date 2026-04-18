#ifndef HARDWARE_VOLTAGE_SENSOR_H
#define HARDWARE_VOLTAGE_SENSOR_H
#include "Common.h"


class IVoltageSensor {
public:
    virtual ~IVoltageSensor() = default;
    virtual const char* SensorName() const = 0;
    virtual bool Initialize() = 0;
    virtual Common::VoltageData ReadData() = 0;
};

#endif  // HARDWARE_VOLTAGE_SENSOR_H
