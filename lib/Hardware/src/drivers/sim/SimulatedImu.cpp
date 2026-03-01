#include "interfaces/IMUInterface.h"

class SimulatedImu : public IIMU {
public:
    bool Initialize();
    Common::IMUData ReadData();
    void Calibrate();
    void setIMUData(const Common::IMUData& data);
private:
    Common::IMUData data_;
};




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
