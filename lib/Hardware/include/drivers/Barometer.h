#ifndef HARDWARE_BAROMETER_DRIVER_H
#define HARDWARE_BAROMETER_DRIVER_H

#include "interfaces/Barometer.h"

class Barometer : public IBarometer {
public:
    bool Initialize() override;
    Common::BarometerData ReadData() override;

private:
    float pressure_ = 0.0f;
    float temperature_ = 0.0f;
};

#endif  // HARDWARE_BAROMETER_DRIVER_H
