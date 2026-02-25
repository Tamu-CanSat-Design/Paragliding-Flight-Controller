#include "HardwareManager.h"
#include "SensorSnapshot.h"

bool HardwareManager::Initialize() {
    bool ok = true;
    ok = barometer_.Initialize() && ok;
    ok = imu_.Initialize() && ok;
    ok = gps_.Initialize() && ok;
    ok = lidar_.Initialize() && ok;
    ok = voltageSensor_.Initialize() && ok;
    ok = radio_.Initialize() && ok;
    ok = audioBeacon_.Initialize() && ok;
    for (int i = 0; i < 2; ++i) {
        ok = cameras_[i].Initialize() && ok;
        ok = releaseServos_[i].Initialize() && ok;
    }
    for (int i = 0; i < 4; ++i) {
        ok = steeringServos_[i].Initialize() && ok;
    }
    return ok;
}

void HardwareManager::ReadSensors() {
    snapshot_.barometer = barometer_.ReadData();
    snapshot_.imu = imu_.ReadData();

    GPSData gps = gps_.ReadData();
    snapshot_.gpsLocation = gps.location;
    snapshot_.gpsTime = gps.time;
    snapshot_.gpsSatellites = gps.satellites;

    snapshot_.rangeMeters = lidar_.ReadData().distanceMeters;
    VoltageData power = voltageSensor_.ReadData();
    snapshot_.voltage = power.voltage;
    snapshot_.current = power.current;
}

const Common::SensorSnapshot& HardwareManager::GetSnapshot() const {
    return snapshot_;
}
