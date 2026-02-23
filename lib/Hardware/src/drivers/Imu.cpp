#include "interfaces/IMUInterface.h"
class IMU : public IIMU {
public:
    bool Initialize();
    Common::IMUData ReadData();
    void Calibrate();
    void setIMUData(const Common::IMUData& data);
private:
    Common::IMUData data_;
};


bool IMU::Initialize() {
    data_ = {};
    return true;
}

Common::IMUData IMU::ReadData() {
    return data_;
}

void IMU::Calibrate() {
}

void IMU::setIMUData(const Common::IMUData& data) {
    data_ = data;
}
