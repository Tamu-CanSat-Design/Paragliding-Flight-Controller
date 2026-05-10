#ifndef HARDWARE_VOLTAGE_SENSOR_DRIVER_H
#define HARDWARE_VOLTAGE_SENSOR_DRIVER_H

#include "../bus/I2CDevice.h"
#include "../interfaces/VoltageSensorInterface.h"

class VoltageSensor : public IVoltageSensor {
public:
    struct Diagnostics {
        uint32_t initFailureCount = 0;
        uint32_t readFailureCount = 0;
        uint32_t staleReturnCount = 0;
    };

    static constexpr const char* kSensorName = "INA260";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::VoltageData ReadData() override;
    Diagnostics GetDiagnostics() const;
    void ClearDiagnostics();

private:
    Common::VoltageData data_{};
    Diagnostics diagnostics_{};
    I2CDevice device_{0x40, Wire};
    bool initialized_ = false;
};

#endif  // HARDWARE_VOLTAGE_SENSOR_DRIVER_H
