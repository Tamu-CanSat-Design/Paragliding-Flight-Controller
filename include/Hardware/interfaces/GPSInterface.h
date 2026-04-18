#ifndef HARDWARE_GPS_H
#define HARDWARE_GPS_H
#include "Common.h"


class IGPS {
public:
    virtual ~IGPS() = default;
    virtual const char* SensorName() const = 0;
    virtual bool Initialize() = 0;
    virtual Common::GPSData ReadData() = 0;
};

#endif // HARDWARE_GPS_H
