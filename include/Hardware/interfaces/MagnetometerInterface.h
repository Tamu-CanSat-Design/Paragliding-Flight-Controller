#ifndef HARDWARE_MAGNETOMETER_INTERFACE_H
#define HARDWARE_MAGNETOMETER_INTERFACE_H

#include "Common.h"

class IMagnetometer {
public:
    virtual ~IMagnetometer() = default;
    virtual const char* SensorName() const = 0;
    virtual bool Initialize() = 0;
    virtual Common::MagData ReadData() = 0;
};

#endif  // HARDWARE_MAGNETOMETER_INTERFACE_H
