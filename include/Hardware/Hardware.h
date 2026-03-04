#pragma once
#include "Hardware/drivers/Barometer.h"
#include "Hardware/drivers/Imu.h"
#include "Hardware/drivers/TeseoGps.h"
#include "Hardware/drivers/Lidar.h"
#include "Hardware/drivers/VoltageSensor.h"
#include "Hardware/drivers/Radio.h" //
#include "Hardware/drivers/AudioBeacon.h"
#include "Hardware/drivers/Camera.h"
#include "Hardware/drivers/ReleaseServo.h"
#include "Hardware/drivers/SteeringServo.h"

#include "bus/I2CDevice.h"


namespace Hardware {

    // Global hardware manager instance. Tasks can access this to read sensors, send radio messages, etc.
    // This is defined here in the .cpp file to avoid circular dependencies with the header.
    

    // Bus definitions
    extern I2CDevice i2c1_; // default address, can be changed by drivers
    extern I2CDevice i2c2_; // default address, can be changed by drivers

    //Sensors:
    extern Barometer barometer_;
    extern IMU imu_;
    extern TeseoGPS gps_;
    extern Lidar lidar_;
    extern VoltageSensor voltageSensor_;
    

    //Actuators
    extern Radio radio_;
    extern AudioBeacon audioBeacon_;
    extern std::array<Camera, 2> cameras_;
    extern std::array<ReleaseServo, 3> mechServos_;
    extern std::array<SteeringServo, 2> steeringServos_;


    bool InitializeAllSensors();
}

