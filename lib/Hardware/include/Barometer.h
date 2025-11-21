#ifndef HARDWARE_BAROMETER_H
#define HARDWARE_BAROMETER_H
#include "Common.h"
class Barometer{
public:
    virtual void Initialize() = 0;
    virtual Common::Vector3 ReadData() = 0;
};

#endif // HARDWARE_BAROMETER_H