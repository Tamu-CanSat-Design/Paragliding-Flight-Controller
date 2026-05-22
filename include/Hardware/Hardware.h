#ifndef HARDWARE_H
#define HARDWARE_H
#include "Hardware/drivers/BarometerOTS.h"
#include "Hardware/drivers/Imu.h"
#include "Hardware/drivers/MagnetometerOTS.h"
#include "Hardware/drivers/TeseoGPS.h"
#include "Hardware/drivers/Lidar.h"
#include "Hardware/drivers/VoltageSensor.h"
#include "Hardware/drivers/AudioBeacon.h"
#include "Hardware/drivers/Camera.h"
#include "Hardware/drivers/Mg90sServo.h"
#include "Hardware/drivers/Sg90Servo.h"
#include "Hardware/drivers/RTC.h"
#include "Commlink.h"
#include "bus/I2CDevice.h"

namespace Hardware {

    // Global hardware manager instance. Tasks can access this to read sensors, send radio messages, etc.
    // This is defined here in the .cpp file to avoid circular dependencies with the header.
    

    // Bus definitions
    extern I2CDevice i2c1_; // default address, can be changed by drivers
    extern I2CDevice i2c2_; // default address, can be changed by drivers

    //Sensors:
    extern BarometerOTS barometer_;
    extern IMU imu_;
    extern MagnetometerOTS magnetometer_;
    extern TeseoGPS gps_;
    extern Lidar lidar_;
    extern VoltageSensor voltageSensor_;
    extern RTC rtc_;

    //Radio
    extern Commlink commlink_;

    //Actuators
    // extern Radio radio_;
    extern AudioBeacon audioBeacon_;
    extern Camera camera_;
    extern Mg90sServo ContainerPayloadServo_;
    extern Mg90sServo EggInstrumentServo_;
    extern Mg90sServo LidarCameraServo_;
    extern Sg90Servo LeftServo_;
    extern Sg90Servo RightServo_;
    

    bool InitializeAllSensors();
}


#endif // HARDWARE_H
