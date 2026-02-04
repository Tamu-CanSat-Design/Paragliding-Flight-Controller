#ifndef HARDWARE_HARDWAREMANAGER_H
#define HARDWARE_HARDWAREMANAGER_H

#include "EEPROM.h"
#include "Common.h"
#include "SensorSnapshot.h"
#include "drivers/Barometer.h"
#include "drivers/Gps.h"
#include "drivers/Imu.h"
#include "drivers/VoltageSensor.h"
#include "drivers/Lidar.h"
#include "drivers/Radio.h"
#include "drivers/AudioBeacon.h"
#include "drivers/Camera.h"
#include "drivers/ReleaseServo.h"
#include "drivers/SteeringServo.h"

class HardwareManager{
public:
    bool Initialize();
    void ReadSensors();
    const SensorSnapshot& GetSnapshot() const;
    
private:
    SensorSnapshot snapshot_;
    Barometer barometer_;
    IMU imu_;
    GPS gps_;
    Lidar lidar_;
    VoltageSensor voltageSensor_;
    Radio radio_;
    AudioBeacon audioBeacon_;
    Camera cameras_[2];
    ReleaseServo releaseServos_[2];
    SteeringServo steeringServos_[4];
};

#endif // HARDWARE_HARDWAREMANAGER_H
