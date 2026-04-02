#include "Hardware/Hardware.h"

namespace Hardware {

    // Global hardware manager instance. Tasks can access this to read sensors, send radio messages, etc.
    // This is defined here in the .cpp file to avoid circular dependencies with the header.
    
    Barometer barometer_{Wire};
    IMU imu_{Wire1};
    TeseoGPS gps_{Wire};
    Lidar lidar_{Wire};
    VoltageSensor voltageSensor_{Wire1};
    Radio radio_;
    AudioBeacon audioBeacon_;
    Camera camera_;
    ReleaseServo ContainerPayloadServo_;
    ReleaseServo EggInstrumentServo_;
    ReleaseServo LidarCameraServo_;
    SteeringServo LeftServo_;
    SteeringServo RightServo_;
    I2CDevice i2c1_{0, Wire};
    I2CDevice i2c2_{0, Wire1};
    
    bool InitializeAllSensors() {
        bool ok = true;
        bool result;

        result = barometer_.Initialize();
        Serial.printf("[HW] %-20s %s\n", barometer_.SensorName(), result ? "OK" : "FAIL");
        ok = result && ok;

        result = imu_.Initialize();
        Serial.printf("[HW] %-20s %s\n", imu_.SensorName(), result ? "OK" : "FAIL");
        ok = result && ok;

        result = gps_.Initialize();
        Serial.printf("[HW] %-20s %s\n", gps_.SensorName(), result ? "OK" : "FAIL");
        ok = result && ok;

        result = lidar_.Initialize();
        Serial.printf("[HW] %-20s %s\n", lidar_.SensorName(), result ? "OK" : "FAIL");
        ok = result && ok;

        result = voltageSensor_.Initialize();
        Serial.printf("[HW] %-20s %s\n", voltageSensor_.SensorName(), result ? "OK" : "FAIL");
        ok = result && ok;

        result = radio_.Initialize();
        Serial.printf("[HW] %-20s %s\n", "Radio", result ? "OK" : "FAIL");
        ok = result && ok;

        result = audioBeacon_.Initialize();
        Serial.printf("[HW] %-20s %s\n", "AudioBeacon", result ? "OK" : "FAIL");
        ok = result && ok;

        result = camera_.Initialize();
        Serial.printf("[HW] %-20s %s\n", "Camera", result ? "OK" : "FAIL");
        ok = result && ok;

        result = ContainerPayloadServo_.Initialize();
        Serial.printf("[HW] %-20s %s\n", "ContainerServo", result ? "OK" : "FAIL");
        ok = result && ok;

        result = EggInstrumentServo_.Initialize();
        Serial.printf("[HW] %-20s %s\n", "EggServo", result ? "OK" : "FAIL");
        ok = result && ok;

        result = LidarCameraServo_.Initialize();
        Serial.printf("[HW] %-20s %s\n", "LidarCameraServo", result ? "OK" : "FAIL");
        ok = result && ok;

        result = LeftServo_.Initialize();
        Serial.printf("[HW] %-20s %s\n", "LeftServo", result ? "OK" : "FAIL");
        ok = result && ok;

        result = RightServo_.Initialize();
        Serial.printf("[HW] %-20s %s\n", "RightServo", result ? "OK" : "FAIL");
        ok = result && ok;

        Serial.printf("[HW] Init complete: %s\n", ok ? "ALL OK" : "ONE OR MORE FAILED");
        return ok;
    }
}
