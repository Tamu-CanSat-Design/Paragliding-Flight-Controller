#include <Arduino.h>

#include "Common.h"
#include "Hardware/drivers/Lidar.h"

namespace {
Lidar gLidar;
constexpr unsigned long kPrintPeriodMs = 250;

const char* LidarStatusName(Common::LidarStatus status) {
    switch (status) {
        case Common::LidarStatus::Uninitialized:
            return "Uninitialized";
        case Common::LidarStatus::Ok:
            return "Ok";
        case Common::LidarStatus::NoAck:
            return "NoAck";
        case Common::LidarStatus::InvalidRange:
            return "InvalidRange";
    }
    return "Unknown";
}
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("lidar_bench: starting");

    const bool ok = gLidar.Initialize();
    Serial.print("lidar_bench: init=");
    Serial.println(ok ? "OK" : "FAIL");
}

void loop() {
    static unsigned long lastPrintMs = 0;
    const unsigned long nowMs = millis();
    if ((nowMs - lastPrintMs) < kPrintPeriodMs) {
        return;
    }
    lastPrintMs = nowMs;

    const Common::LidarData data = gLidar.ReadData();
    Serial.print("lidar_bench: distance_m=");
    Serial.print(data.distanceMeters, 3);
    Serial.print(" valid=");
    Serial.print(data.valid ? "true" : "false");
    Serial.print(" status=");
    Serial.print(LidarStatusName(data.status));
    Serial.print(" failures=");
    Serial.println(data.failureCount);
}
