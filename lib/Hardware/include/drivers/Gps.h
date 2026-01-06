#ifndef HARDWARE_GPS_DRIVER_H
#define HARDWARE_GPS_DRIVER_H

#include "interfaces/GPS.h"

class GPS : public IGPS {
public:
    bool Initialize() override;
    GPSData ReadData() override;
};

#endif  // HARDWARE_GPS_DRIVER_H
