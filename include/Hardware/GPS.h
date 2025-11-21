#ifndef HARDWARE_GPS_H
#define HARDWARE_GPS_H
#include "Common.h"

class IGPS {
public:
    virtual void Initialize() = 0;
    virtual void ReadData() = 0;
    virtual void setGPSData(const Common::Vector3& location, float time, int satellites) = 0;
};

#endif // HARDWARE_GPS_H