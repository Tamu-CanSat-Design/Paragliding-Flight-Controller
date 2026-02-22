#ifndef HARDWARE_IMU_INTERFACE_H
#define HARDWARE_IMU_INTERFACE_H

#include "Common.h"

using IMUData = Common::IMUData;

class IIMU {
public:
    virtual ~IIMU() = default;
    virtual const char* SensorName() const = 0;
    virtual bool Initialize() = 0;
    virtual IMUData ReadData() = 0;
    virtual void Calibrate() = 0;
    virtual void setIMUData(const IMUData& data) = 0;
};

#endif  // HARDWARE_IMU_INTERFACE_H
