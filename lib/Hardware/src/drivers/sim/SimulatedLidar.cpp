#include "drivers/sim/SimulatedLidar.h"

bool SimulatedLidar::Initialize() {
    data_.distanceMeters = 0.0f;
    return true;
}

LidarData SimulatedLidar::ReadData() {
    return data_;
}
