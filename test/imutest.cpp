#include <Arduino.h>

#include "Common.h"
#include "Hardware/drivers/Imu.h"

namespace {

IMU gImu;

void PrintVector3(const Common::Vector3& vec) {
    Serial.print("{");
    Serial.print(vec.x, 4);
    Serial.print(", ");
    Serial.print(vec.y, 4);
    Serial.print(", ");
    Serial.print(vec.z, 4);
    Serial.print("}");
}

void PrintImuData(const Common::IMUData& d) {
    Serial.print("gyro=");
    PrintVector3(d.gyroVel);
    Serial.print(" accel=");
    PrintVector3(d.acceleration);
    Serial.print(" temp=");
    Serial.print(d.temperatureCelsius, 2);
    Serial.println();
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);

    const bool imuOk = gImu.Initialize();
    Serial.print("IMU init: ");
    Serial.println(imuOk ? "OK" : "FAIL");
    if (!imuOk) {
        while (true) {
            delay(1000);
        }
    }
}

void loop() {
    // Drain FIFO each iteration; fall back to direct read when FIFO is empty.
    Common::IMUData fifoSamples[IMU::kFifoWatermarkPairs];
    const size_t fifoCount = gImu.ReadFifoSamples(fifoSamples, IMU::kFifoWatermarkPairs);

    if (fifoCount > 0) {
        Serial.print("[FIFO] ");
        Serial.print(fifoCount);
        Serial.println(" sample(s):");
        for (size_t i = 0; i < fifoCount; ++i) {
            Serial.print("  [");
            Serial.print(i);
            Serial.print("] ");
            PrintImuData(fifoSamples[i]);
        }

        const IMU::FifoDiagnostics diag = gImu.GetFifoDiagnostics();
        if (diag.overrunCount > 0 || diag.truncatedPairCount > 0 || diag.unexpectedTagCount > 0) {
            Serial.print("[FIFO diag] overruns=");
            Serial.print(diag.overrunCount);
            Serial.print(" truncated=");
            Serial.print(diag.truncatedPairCount);
            Serial.print(" unexpectedTags=");
            Serial.println(diag.unexpectedTagCount);
        }
    } else {
        // No FIFO data available; fall back to direct register read.
        Serial.print("[direct] ");
        PrintImuData(gImu.ReadData());
    }

    delay(10);
}