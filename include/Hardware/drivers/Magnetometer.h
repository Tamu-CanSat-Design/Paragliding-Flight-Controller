#ifndef HARDWARE_MAGNETOMETER_DRIVER_H
#define HARDWARE_MAGNETOMETER_DRIVER_H

#include "../bus/I2CDevice.h"
#include "../interfaces/MagnetometerInterface.h"

class Magnetometer : public IMagnetometer {
public:
    struct Diagnostics {
        uint32_t initFailureCount = 0;
        uint32_t readFailureCount = 0;
        uint32_t staleReturnCount = 0;
    };

    static constexpr const char* kSensorName = "IIS2MDC";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::MagData ReadData() override;
    Diagnostics GetDiagnostics() const;
    void ClearDiagnostics();

private:
    Common::MagData data_{};
    Diagnostics diagnostics_{};
    I2CDevice device_{0x1E, Wire1};
    bool initialized_ = false;
};

#endif  // HARDWARE_MAGNETOMETER_DRIVER_H
