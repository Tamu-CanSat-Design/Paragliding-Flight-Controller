#include <Arduino.h>

#include "Common.h"
#include "Hardware/drivers/Imu.h"

namespace {

IMU gImu;


    void PrintVector3(const Common::Vector3& vec) {
        Serial.print("{");
        Serial.print(vec.x, 2);
        Serial.print(", ");
        Serial.print(vec.y, 2);
        Serial.print(", ");
        Serial.print(vec.z, 2);
        Serial.print("}");

    }  
}// namespace

void setup() {
    Serial.begin(115200);
    delay(1000);

    const bool imuOk = gImu.Initialize();
    Serial.print("IMU init: ");
    Serial.println(imuOk ? "OK" : "FAIL");
    //suspend if failed
    if (!imuOk) {
        while (true) {
            delay(1000);
        }
    }

   
   
}

void loop() {
    static unsigned long lastPrintMs = 0;
    const unsigned long nowMs = millis();
    
    lastPrintMs = nowMs;

    const Common::IMUData sample = gImu.ReadData();

    Serial.print("gyro=");
    PrintVector3(sample.gyroVel);
    Serial.print(" accel=");
    PrintVector3(sample.acceleration);
    Serial.println();
    Serial.print("temp=");
    Serial.print(sample.temperatureCelsius, 4);


}