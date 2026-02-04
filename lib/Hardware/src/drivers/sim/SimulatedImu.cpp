#include "drivers/sim/SimulatedImu.h"

bool SimulatedImu::Initialize() {
    data_ = {};
    return true;
}

IMUData SimulatedImu::ReadData() {
    return data_;
}

void SimulatedImu::Calibrate() {
}

void SimulatedImu::setIMUData(const IMUData& data) {
    data_ = data;
}
