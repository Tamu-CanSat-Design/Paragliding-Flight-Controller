#ifndef HARDWARE_IMU_DRIVER_H
#define HARDWARE_IMU_DRIVER_H

#include <cstdint>

#include "../bus/I2CDevice.h"
#include "../interfaces/IMUInterface.h"

class IMU : public IIMU {
public:
    struct FifoDiagnostics {
        uint32_t overrunCount = 0;
        uint32_t nearFullCount = 0;
        uint32_t unexpectedTagCount = 0;
        uint32_t truncatedPairCount = 0;
    };

    static constexpr const char* kSensorName = "LSM6DSR + LIS2MDL";
    static constexpr float kSamplePeriodSeconds = 1.0f / 416.0f;
    static constexpr size_t kFifoWatermarkPairs = 4;

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::IMUData ReadData() override;
    size_t ReadFifoSamples(Common::IMUData* outSamples, size_t maxSamples);
    Common::MagData ReadMagData();
    FifoDiagnostics GetFifoDiagnostics() const;
    void ClearFifoDiagnostics();
    void Calibrate() override;

private:
    // Cache of the most recent IMU sample.
    Common::IMUData data_{};
    Common::MagData magData_{};
    FifoDiagnostics fifoDiagnostics_{};
    // Biases are computed in Calibrate() and subtracted on each read.
    Common::Vector3 accelBias_{};
    Common::Vector3 gyroBias_{};
    Common::Vector3 magBias_{};
    // Separate bus handles keep address/protocol state per sensor die.
    I2CDevice imuDevice_{};
    I2CDevice magDevice_{};
    // Guards against reads before hardware is configured.
    bool initialized_{false};
};

#endif  // HARDWARE_IMU_DRIVER_H
