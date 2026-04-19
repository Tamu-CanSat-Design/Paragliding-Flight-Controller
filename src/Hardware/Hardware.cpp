#include "Hardware/Hardware.h"

namespace Hardware {

    // Global hardware manager instance. Tasks can access this to read sensors, send radio messages, etc.
    // This is defined here in the .cpp file to avoid circular dependencies with the header.
    
    Barometer barometer_;
    IMU imu_;
    TeseoGPS gps_;
    Lidar lidar_;
    VoltageSensor voltageSensor_;
    Radio radio_;
    AudioBeacon audioBeacon_;
    Camera camera_;
    ReleaseServo ContainerPayloadServo_;
    ReleaseServo EggInstrumentServo_;
    ReleaseServo LidarCameraServo_;
    SteeringServo LeftServo_;
    SteeringServo RightServo_;
    I2CDevice i2c1_{};
    I2CDevice i2c2_{};
    



    bool InitializeAllSensors() {
        bool ok = true;
        ok = barometer_.Initialize() && ok;
        ok = imu_.Initialize() && ok;
        ok = gps_.Initialize() && ok;
        ok = lidar_.Initialize() && ok;
        ok = voltageSensor_.Initialize() && ok;
        ok = radio_.Initialize() && ok;
        ok = audioBeacon_.Initialize() && ok;
        ok = camera_.Initialize() && ok;

        ok = EggInstrumentServo_.Initialize() && ok;
        ok = LidarCameraServo_.Initialize() && ok;
        ok = ContainerPayloadServo_.Initialize() && ok;
        ok = LeftServo_.Initialize() && ok;
        ok = RightServo_.Initialize() && ok;
        return ok;
    }
}


