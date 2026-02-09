#ifndef HARDWARE_GPS_H
#define HARDWARE_GPS_H
#include "Common.h"

struct GPSData {
    Common::GpsData location;
    float time;
    int satellites;
};

class IGPS {
public:
    virtual ~IGPS() = default;
    virtual bool Initialize() = 0;
    virtual GPSData ReadData() = 0;
};

#endif // HARDWARE_GPS_H
