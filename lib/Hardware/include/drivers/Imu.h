#ifndef HARDWARE_IMU_DRIVER_H
#define HARDWARE_IMU_DRIVER_H

#include <cstdint>

#include "bus/I2CDevice.h"
#include "interfaces/IMU.h"

class IMU : public IIMU {
public:
    static constexpr const char* kSensorName = "LSM6DSR + LIS2MDL";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    IMUData ReadData() override;
    void Calibrate() override;
    void setIMUData(const IMUData& data) override;

private:
    // Cache of the most recent IMU sample.
    IMUData data_{};
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
