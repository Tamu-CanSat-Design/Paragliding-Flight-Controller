#include "Hardware/Hardware.h"

namespace Hardware {

    // Global hardware manager instance. Tasks can access this to read sensors, send radio messages, etc.
    // This is defined here in the .cpp file to avoid circular dependencies with the header.
    
    BarometerOTS barometer_;
    IMU imu_;
    MagnetometerOTS magnetometer_;
    TeseoGPS gps_;
    Lidar lidar_;
    VoltageSensor voltageSensor_;
    Commlink commlink_;

    AudioBeacon audioBeacon_;
    RTC rtc_;
    Camera camera_;
    Mg90sServo ContainerPayloadServo_{15};
    Mg90sServo EggInstrumentServo_{14};
    Mg90sServo LidarCameraServo_{13};
    Sg90Servo LeftServo_{23};
    Sg90Servo RightServo_{22};
    I2CDevice i2c1_{};
    I2CDevice i2c2_{};
    



    bool InitializeAllSensors() {
        bool ok = true;
        
        ok = imu_.Initialize() && ok;
        Serial.println("IMU");
        Serial.println(ok ? "true" : "false");
        ok = magnetometer_.Initialize() && ok;
        Serial.println("Magnetometer");
        Serial.println(ok ? "true" : "false");
        ok = barometer_.Initialize() && ok;
        Serial.println("Barometer");
        Serial.println(ok ? "true" : "false");

        ok = gps_.Initialize() && ok;
        Serial.println("GPS");
        Serial.println(ok ? "true" : "false");
#if CANSAT_ENABLE_LIDAR
        ok = lidar_.Initialize() && ok;
        
#else
        // LiDAR temporarily disabled until mechanical LiDAR leg is complete.
        Serial.println("Hardware: LiDAR disabled");
#endif
        ok = voltageSensor_.Initialize() && ok;
        Serial.println("Voltage");
        Serial.println(ok ? "true" : "false");
        ok = rtc_.Initialize() && ok;
        Serial.println("RTC");
        Serial.println(ok ? "true" : "false");
        ok = commlink_.Initialize() && ok;
        Serial.println("Commlink");
        Serial.println(ok ? "true" : "false");
        {
                bool r = audioBeacon_.Initialize();
                Serial.print("Hardware: Audio beacon initialized: ");
                Serial.println(r ? "true" : "false");
                ok = r && ok;
        }
                ok = camera_.Initialize() && ok;

        // ok = EggInstrumentServo_.Initialize() && ok;
        // ok = LidarCameraServo_.Initialize() && ok;
        // ok = ContainerPayloadServo_.Initialize() && ok;
        // ok = LeftServo_.Initialize() && ok;
        // ok = RightServo_.Initialize() && ok;
        return ok;
    }
}
