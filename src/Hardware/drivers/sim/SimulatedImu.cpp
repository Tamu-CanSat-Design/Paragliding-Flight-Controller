#include "Hardware/drivers/sim/SimulatedImu.h"

bool SimulatedImu::Initialize() {
    data_ = {};
    return true;
}

Common::IMUData SimulatedImu::ReadData() {
    return data_;
}

void SimulatedImu::Calibrate() {
}

void SimulatedImu::setIMUData(const Common::IMUData& data) {
    data_ = data;
}
