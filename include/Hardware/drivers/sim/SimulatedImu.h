#ifndef HARDWARE_SIMULATED_IMU_H
#define HARDWARE_SIMULATED_IMU_H

#include "interfaces/IMUInterface.h"

class SimulatedImu : public IIMU {
public:
    static constexpr const char* kSensorName = "Simulated IMU";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    IMUData ReadData() override;
    void Calibrate() override;
    void setIMUData(const IMUData& data) override;

private:
    IMUData data_{};
};

#endif  // HARDWARE_SIMULATED_IMU_H
