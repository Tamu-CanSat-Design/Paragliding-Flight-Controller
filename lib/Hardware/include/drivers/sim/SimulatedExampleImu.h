#ifndef HARDWARE_SIMULATED_EXAMPLE_IMU_H
#define HARDWARE_SIMULATED_EXAMPLE_IMU_H

#include "interfaces/IMU.h"

// Example simulated IMU. Real simulated drivers should follow this pattern.
class SimulatedExampleImu : public IIMU {
public:
    static constexpr const char* kSensorName = "Simulated Example IMU";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    IMUData ReadData() override;
    void Calibrate() override;
    void setIMUData(const IMUData& data) override;

private:
    IMUData data_{};
};

#endif  // HARDWARE_SIMULATED_EXAMPLE_IMU_H
