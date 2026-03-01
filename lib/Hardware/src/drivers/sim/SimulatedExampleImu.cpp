#include "interfaces/IMUInterface.h"

class SimulatedExampleImu : public IIMU {
public:
    bool Initialize();
    Common::IMUData ReadData();
    void Calibrate();
    void setIMUData(const Common::IMUData& data);
private:
    Common::IMUData data_;
};


bool SimulatedExampleImu::Initialize() {
    // PSEUDOCODE (simulation setup):
    // In simulation, we define an initial state instead of talking to hardware.
    // simTime_ = 0.0f;
    // angularRate_ = {0.0f, 0.0f, 0.0f};
    // accel_ = {0.0f, 0.0f, 9.81f};
    // seed_ = 1234;
    data_ = {};
    return true;
}

Common::IMUData SimulatedExampleImu::ReadData() {
    // PSEUDOCODE (simple motion model):
    // This produces predictable motion so tests can assert expected values.
    // simTime_ += 0.01f;
    // float yaw = 0.1f * sinf(simTime_);
    // float pitch = 0.05f * cosf(simTime_);
    // float roll = 0.0f;
    //
    // data_.eulerAngles = {roll, pitch, yaw}; // Orientation in radians.
    // data_.gyroVel = {0.0f, 0.0f, 0.1f * cosf(simTime_)}; // Angular rate.
    // data_.acceleration = {0.0f, 0.0f, 9.81f}; // Gravity-only accel.
    // data_.magneticField = {1.0f, 0.0f, 0.0f}; // Fixed magnetic vector.
    return data_;
}

void SimulatedExampleImu::Calibrate() {
    // PSEUDOCODE:
    // In simulation, calibration often just resets offsets.
    // accelBias_ = {};
    // gyroBias_ = {};
}

void SimulatedExampleImu::setIMUData(const Common::IMUData& data) {
    // Allow tests to inject simulated values directly without a motion model.
    data_ = data;
}
