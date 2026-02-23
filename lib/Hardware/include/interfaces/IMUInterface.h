#ifndef HARDWARE_IMU_INTERFACE_H
#define HARDWARE_IMU_INTERFACE_H
#include "Common.h"


class IIMU {
public:
    virtual ~IIMU() = default;
    virtual bool Initialize() = 0;
    virtual Common::IMUData ReadData() = 0;
    virtual void Calibrate() = 0;
    virtual void setIMUData(const Common::IMUData& data) = 0;

};

#endif // HARDWARE_IMU_INTERFACE_H
