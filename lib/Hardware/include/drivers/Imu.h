#ifndef HARDWARE_IMU_DRIVER_H
#define HARDWARE_IMU_DRIVER_H

#include "interfaces/IMU.h"

class IMU : public IIMU {
public:
    bool Initialize() override;
    IMUData ReadData() override;
    void Calibrate() override;
    void setIMUData(const IMUData& data) override;

private:
    IMUData data_{};
};

#endif  // HARDWARE_IMU_DRIVER_H
