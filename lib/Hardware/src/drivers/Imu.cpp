#include "drivers/Imu.h"

bool IMU::Initialize() {
    data_ = {};
    return true;
}

IMUData IMU::ReadData() {
    return data_;
}

void IMU::Calibrate() {
}

void IMU::setIMUData(const IMUData& data) {
    data_ = data;
}
