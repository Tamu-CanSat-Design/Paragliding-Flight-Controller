#include "Hardware/Hardware.h"

namespace Hardware {

    // Global hardware manager instance. Tasks can access this to read sensors, send radio messages, etc.
    // This is defined here in the .cpp file to avoid circular dependencies with the header.
    
    Barometer barometer_;
    IMU imu_;
    MagnetometerOTS magnetometer_;
    TeseoGPS gps_;
    Lidar lidar_;
    VoltageSensor voltageSensor_;
    Commlink commlink_;

    AudioBeacon audioBeacon_;
    RTC rtc_;
    Camera camera_;
    ReleaseServo ContainerPayloadServo_{/* TODO: pin */ 2};
    ReleaseServo EggInstrumentServo_{/* TODO: pin */ 3};
    ReleaseServo LidarCameraServo_{/* TODO: pin */ 4};
    SteeringServo LeftServo_{[]{ SteeringServo::Config c; c.pin = /* TODO: pin */ 5; return c; }()};
    SteeringServo RightServo_{[]{ SteeringServo::Config c; c.pin = /* TODO: pin */ 6; return c; }()};
    I2CDevice i2c1_{};
    I2CDevice i2c2_{};
    



    bool InitializeAllSensors() {
        bool ok = true;
        ok = barometer_.Initialize() && ok;
        ok = imu_.Initialize() && ok;
        ok = magnetometer_.Initialize() && ok;
        ok = gps_.Initialize() && ok;
#if CANSAT_ENABLE_LIDAR
        ok = lidar_.Initialize() && ok;
#else
        // LiDAR temporarily disabled until mechanical LiDAR leg is complete.
        Serial.println("Hardware: LiDAR disabled");
#endif
        ok = voltageSensor_.Initialize() && ok;
        // ok = radio_.Initialize() && ok;
        ok = rtc_.Initialize() && ok;
        // ok = audioBeacon_.Initialize() && ok;
        // ok = camera_.Initialize() && ok;

        // ok = EggInstrumentServo_.Initialize() && ok;
        // ok = LidarCameraServo_.Initialize() && ok;
        // ok = ContainerPayloadServo_.Initialize() && ok;
        // ok = LeftServo_.Initialize() && ok;
        // ok = RightServo_.Initialize() && ok;
        return ok;
    }
}
