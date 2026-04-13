#include "Hardware/drivers/sim/SimulatedGps.h"

bool SimulatedGps::Initialize() {
    data_ = {};
    return true;
}

Common::GPSData SimulatedGps::ReadData() {
    return data_;
}
