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
    std::array<Camera, 2> cameras_;
    std::array<ReleaseServo, 3> mechServos_;
    std::array<SteeringServo, 2> steeringServos_;
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
}


