#ifndef HARDWARE_BAROMETEROTS_DRIVER_H
#define HARDWARE_BAROMETEROTS_DRIVER_H

#include <cstdint>

#include "../interfaces/BarometerInterface.h"
#include "Common.h"
#include "driver_bmp390.h"

class BarometerOTS : public IBarometer {
public:
    static constexpr const char* kSensorName = "BMP390";

    struct Diagnostics {
        // Init phase
        uint32_t initFailureCount = 0;
        uint8_t  lastInitErrorCode = 0;  // bmp390_init() return value on last failure (1=I2C, 4=bad chip ID, 5=reset fail, 6=calib fail)
        uint8_t  foundAddress = 0;       // 7-bit I2C address confirmed during probe (0 if init never succeeded)

        // Read phase
        uint32_t totalReadCount = 0;
        uint32_t readFailureCount = 0;
        uint32_t consecutiveReadFailures = 0;
        uint32_t maxConsecutiveReadFailures = 0;
        uint8_t  lastReadErrorCode = 0;  // bmp390_read_temperature_pressure() return value on last failure
        uint32_t staleReturnCount = 0;   // times ReadData() returned cached data (uninitialised or failed read)
    };

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::BarometerData ReadData() override;
    bool SetCalibrationPressure(float pressureHpa);
    Diagnostics GetDiagnostics() const;

private:
    float PressureToRelativeAltitude(float pressureHpa) const;

    bmp390_handle_t handle_{};
    Common::BarometerData data_{};
    Diagnostics diagnostics_{};
    float referencePressureHpa_ = 1013.25f;
    bool initialized_ = false;
};

#endif  // HARDWARE_BAROMETEROTS_DRIVER_H
