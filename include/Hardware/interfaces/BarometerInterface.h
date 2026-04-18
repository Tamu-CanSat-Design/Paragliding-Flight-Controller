#ifndef HARDWARE_BAROMETER_H
#define HARDWARE_BAROMETER_H
#include "Common.h"

class IBarometer{
public:
    virtual ~IBarometer() = default;
    virtual const char* SensorName() const = 0;
    virtual bool Initialize() = 0;
    virtual Common::BarometerData ReadData() = 0;
};

#endif // HARDWARE_BAROMETER_H
