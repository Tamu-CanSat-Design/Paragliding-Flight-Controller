#include "HardwareManager.h"

bool HardwareManager::Initialize() {
    bool ok = true;
    ok = barometer_.Initialize() && ok;
    ok = imu_.Initialize() && ok;
    ok = gps_.Initialize() && ok;
    ok = lidar_.Initialize() && ok;
    ok = voltageSensor_.Initialize() && ok;
    ok = radio_.Initialize() && ok;
    ok = audioBeacon_.Initialize() && ok;
    for (size_t i = 0; i < cameras_.size(); ++i) {
        ok = cameras_[i].Initialize() && ok;
    }

    for (size_t i = 0; i < mechServos_.size(); ++i) {
        ok = mechServos_[i].Initialize() && ok;
    }

    for (size_t i = 0; i < steeringServos_.size(); ++i) {
        ok = steeringServos_[i].Initialize() && ok;
    }
    return ok;
}

void HardwareManager::ReadSensors() {
    
    snapshot_.barometer = barometer_.ReadData();
    snapshot_.imu = imu_.ReadData();

    Common::GPSData gps = gps_.ReadData();
    snapshot_.gpsData = gps;

    snapshot_.lidarData = lidar_.ReadData();
    Common::VoltageData power = voltageSensor_.ReadData();
    snapshot_.voltage = power.voltage;
    snapshot_.current = power.current;
}

const Common::SensorSnapshot& HardwareManager::GetSnapshot() const {
    return snapshot_;
}
