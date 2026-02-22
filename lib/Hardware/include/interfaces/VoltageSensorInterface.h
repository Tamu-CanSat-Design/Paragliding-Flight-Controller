#ifndef HARDWARE_VOLTAGE_SENSOR_H
#define HARDWARE_VOLTAGE_SENSOR_H

struct VoltageData {
    float voltage;
    float current;
    float power;
};

class IVoltageSensor {
public:
    virtual ~IVoltageSensor() = default;
    virtual const char* SensorName() const = 0;
    virtual bool Initialize() = 0;
    virtual VoltageData ReadData() = 0;
};

#endif  // HARDWARE_VOLTAGE_SENSOR_H
