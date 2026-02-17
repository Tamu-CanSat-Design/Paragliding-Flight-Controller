#ifndef HARDWARE_HARDWAREMANAGER_H
#define HARDWARE_HARDWAREMANAGER_H

#include "EEPROM.h"
#include "Common.h"
#include "interfaces/BarometerInterface.h"
#include "interfaces/GpsInterface.h"
#include "interfaces/ImuInterface.h"
#include "interfaces/VoltageSensorInterface.h"
#include "interfaces/LidarInterface.h"
#include "interfaces/RadioInterface.h"
#include "interfaces/AudioBeaconInterface.h"
#include "interfaces/CameraInterface.h"
#include "interfaces/ServoInterface.h"

class HardwareManager{
public:
    bool Initialize();
    void ReadSensors();
    const Common::SensorSnapshot& GetSnapshot() const;
    
private:
    Common::SensorSnapshot snapshot_;
    IBarometer barometer_;
    IIMU imu_;
    IGPS gps_;
    ILidar lidar_;
    IVoltageSensor voltageSensor_;
    IRadio radio_;
    IAudioBeacon audioBeacon_;
    ICamera cameras_[2];
    IServo mechServos_[3];
    IServo steeringServos_[2];
};

#endif // HARDWARE_HARDWAREMANAGER_H
