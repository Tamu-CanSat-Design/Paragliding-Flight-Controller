#ifndef HARDWARE_SIMULATED_IMU_H
#define HARDWARE_SIMULATED_IMU_H

#include "interfaces/IMU.h"

class SimulatedImu : public IIMU {
public:
    bool Initialize() override;
    IMUData ReadData() override;
    void Calibrate() override;
    void setIMUData(const IMUData& data) override;

private:
    IMUData data_{};
};

#endif  // HARDWARE_SIMULATED_IMU_H
