#ifndef HARDWARE_BAROMETERBMP3_DRIVER_H
#define HARDWARE_BAROMETERBMP3_DRIVER_H

#include <cstdint>

#include "../interfaces/BarometerInterface.h"
#include "Common.h"
#include "bmp3.h"

class BarometerBMP3 : public IBarometer {
public:
    static constexpr const char* kSensorName = "BMP390-BMP3";

    struct Diagnostics {
        uint32_t initFailureCount = 0;
        int8_t primaryInitApiResult = 0;
        int8_t primaryInitInterfaceResult = 0;
        uint8_t primaryInitChipId = 0;
        int8_t secondaryInitApiResult = 0;
        int8_t secondaryInitInterfaceResult = 0;
        uint8_t secondaryInitChipId = 0;
        int8_t lastInitApiResult = 0;
        int8_t lastInitInterfaceResult = 0;
        uint8_t lastInitChipId = 0;
        int8_t lastConfigApiResult = 0;
        int8_t lastConfigInterfaceResult = 0;
        uint8_t foundAddress = 0;
        uint8_t chipId = 0;

        uint32_t totalReadCount = 0;
        uint32_t readFailureCount = 0;
        uint32_t consecutiveReadFailures = 0;
        uint32_t maxConsecutiveReadFailures = 0;
        uint32_t notReadyCount = 0;
        uint32_t staleReturnCount = 0;
        int8_t lastReadApiResult = 0;
        int8_t lastReadInterfaceResult = 0;
    };

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::BarometerData ReadData() override;
    bool SetCalibrationPressure(float pressureHpa);
    Diagnostics GetDiagnostics() const;

private:
    bool TryInitializeAtAddress(uint8_t address);
    bool ConfigureSensor();
    void RecordReadFailure(int8_t apiResult);
    float PressureToRelativeAltitude(float pressureHpa) const;

    bmp3_dev device_{};
    bmp3_settings settings_{};
    Common::BarometerData data_{};
    Diagnostics diagnostics_{};
    float referencePressureHpa_ = 1013.25f;
    uint8_t i2cAddress_ = BMP3_ADDR_I2C_PRIM;
    bool initialized_ = false;
};

#endif  // HARDWARE_BAROMETERBMP3_DRIVER_H
