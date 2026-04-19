#ifndef HARDWARE_IMU_INTERFACE_H
#define HARDWARE_IMU_INTERFACE_H

#include "Common.h"


class IIMU {
public:
    virtual ~IIMU() = default;
    virtual const char* SensorName() const = 0;
    virtual bool Initialize() = 0;
    virtual Common::IMUData ReadData() = 0;
    virtual void Calibrate() = 0;

};

#endif  // HARDWARE_IMU_INTERFACE_H
