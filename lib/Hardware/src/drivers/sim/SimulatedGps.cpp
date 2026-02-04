#include "drivers/sim/SimulatedGps.h"

bool SimulatedGps::Initialize() {
    data_.location = {0.0f, 0.0f, 0.0f};
    data_.time = 0.0f;
    data_.satellites = 0;
    return true;
}

GPSData SimulatedGps::ReadData() {
    return data_;
}
