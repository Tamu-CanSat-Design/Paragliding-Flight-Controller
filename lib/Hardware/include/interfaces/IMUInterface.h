#ifndef HARDWARE_IMU_INTERFACE_H
#define HARDWARE_IMU_INTERFACE_H
#include "Common.h"


struct IMUData {
    Common::Vector3 gyroVel;
    Common::Vector3 acceleration;
    Common::Vector3 magneticField;
    Common::Vector3 eulerAngles;
};
class IIMU {
public:
    virtual ~IIMU() = default;
    virtual bool Initialize() = 0;
    virtual IMUData ReadData() = 0;
    virtual void Calibrate() = 0;
    virtual void setIMUData(const IMUData& data) = 0;

};

#endif // HARDWARE_IMU_INTERFACE_H
