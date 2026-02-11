#ifndef HARDWARE_SIMULATED_GPS_H
#define HARDWARE_SIMULATED_GPS_H

#include "interfaces/GPS.h"

class SimulatedGps : public IGPS {
public:
    static constexpr const char* kSensorName = "Simulated GPS";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    GPSData ReadData() override;

private:
    GPSData data_{};
};

#endif  // HARDWARE_SIMULATED_GPS_H
