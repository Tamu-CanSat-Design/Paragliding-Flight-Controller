#ifndef HARDWARE_VOLTAGE_SENSOR_DRIVER_H
#define HARDWARE_VOLTAGE_SENSOR_DRIVER_H

#include <cstdint>

#include "../bus/I2CDevice.h"
#include "../interfaces/VoltageSensorInterface.h"

class VoltageSensor : public IVoltageSensor {
public:
    static constexpr const char* kSensorName = "INA260";

    explicit VoltageSensor(TwoWire& wire = Wire);
    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::VoltageData ReadData() override;

private:
    bool ProbeAndSelectAddress();
    bool Configure();
    bool WriteRegister16(uint8_t reg, uint16_t value) const;
    bool ReadRegister16(uint8_t reg, uint16_t* value) const;

    I2CDevice device_{};
    Common::VoltageData data_{};
    bool initialized_ = false;
};

#endif  // HARDWARE_VOLTAGE_SENSOR_DRIVER_H
