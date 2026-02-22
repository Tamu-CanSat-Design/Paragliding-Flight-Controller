#ifndef HARDWARE_HARDWAREMANAGER_H
#define HARDWARE_HARDWAREMANAGER_H

#include <array>

#include "Common.h"
#include "drivers/AudioBeacon.h"
#include "drivers/Barometer.h"
#include "drivers/Camera.h"
#include "drivers/Gps.h"
#include "drivers/Imu.h"
#include "drivers/Lidar.h"
#include "drivers/Radio.h"
#include "drivers/ReleaseServo.h"
#include "drivers/SteeringServo.h"
#include "drivers/VoltageSensor.h"

class HardwareManager {
public:
    bool Initialize();
    void ReadSensors();
    const Common::SensorSnapshot& GetSnapshot() const;

private:
    Common::SensorSnapshot snapshot_;
    Barometer barometer_;
    IMU imu_;
    GPS gps_;
    Lidar lidar_;
    VoltageSensor voltageSensor_;
    Radio radio_;
    AudioBeacon audioBeacon_;
    std::array<Camera, 2> cameras_{};
    std::array<ReleaseServo, 3> mechServos_{};
    std::array<SteeringServo, 2> steeringServos_{};
};

#endif  // HARDWARE_HARDWAREMANAGER_H
