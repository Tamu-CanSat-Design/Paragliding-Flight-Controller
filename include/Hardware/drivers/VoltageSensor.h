#ifndef HARDWARE_VOLTAGE_SENSOR_DRIVER_H
#define HARDWARE_VOLTAGE_SENSOR_DRIVER_H

#include "../interfaces/VoltageSensorInterface.h"

class VoltageSensor : public IVoltageSensor {
public:
    static constexpr const char* kSensorName = "Generic Voltage Sensor";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::VoltageData ReadData() override;
};

#endif  // HARDWARE_VOLTAGE_SENSOR_DRIVER_H
