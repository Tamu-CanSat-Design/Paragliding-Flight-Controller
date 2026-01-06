#ifndef HARDWARE_VOLTAGE_SENSOR_DRIVER_H
#define HARDWARE_VOLTAGE_SENSOR_DRIVER_H

#include "interfaces/VoltageSensor.h"

class VoltageSensor : public IVoltageSensor {
public:
    bool Initialize() override;
    VoltageData ReadData() override;
};

#endif  // HARDWARE_VOLTAGE_SENSOR_DRIVER_H
