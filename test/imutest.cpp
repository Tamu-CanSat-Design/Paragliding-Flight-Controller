#include <Arduino.h>
#include <SD.h>

#include "Common.h"
#include "Hardware/drivers/Imu.h"

namespace {

IMU gImu;
File gLogFile;
bool gSdOk = false;

constexpr unsigned long kPrintPeriodMs = 2;
constexpr int kSdChipSelect = BUILTIN_SDCARD;
constexpr const char* kLogFilename = "imu_log.csv";



void LogVector3(const Common::Vector3& v) {
    gLogFile.print(v.x, 6);
    gLogFile.print(',');
    gLogFile.print(v.y, 6);
    gLogFile.print(',');
    gLogFile.print(v.z, 6);
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);

    const bool imuOk = gImu.Initialize();
    Serial.print("IMU init: ");
    Serial.println(imuOk ? "OK" : "FAIL");

    if (!SD.begin(kSdChipSelect)) {
        Serial.println("SD init: FAIL");
        return;
    }

    gLogFile = SD.open(kLogFilename, FILE_WRITE);
    if (!gLogFile) {
        Serial.print("SD open failed: ");
        Serial.println(kLogFilename);
        return;
    }

    gSdOk = true;
    gLogFile.println("timestamp_us,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");
    gLogFile.flush();
    Serial.print("SD logging to: ");
    Serial.println(kLogFilename);
}

void loop() {
    static unsigned long lastPrintMs = 0;
    const unsigned long nowMs = millis();
    if ((nowMs - lastPrintMs) < kPrintPeriodMs) {
        return;
    }
    lastPrintMs = nowMs;

    const Common::IMUData sample = gImu.ReadData();

    // Serial.print("gyro=");
    // PrintVector3(sample.gyroVel);
    // Serial.print(" accel=");
    // PrintVector3(sample.acceleration);
    // Serial.println();

    if (gSdOk) {
        gLogFile.print(millis());  // timestamp in microseconds
        gLogFile.print(',');
        LogVector3(sample.acceleration);
        gLogFile.print(',');
        LogVector3(sample.gyroVel);
        gLogFile.println();
        gLogFile.flush();
    }


}