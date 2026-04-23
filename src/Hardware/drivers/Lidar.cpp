#include "Hardware/drivers/Lidar.h"

#include <Wire.h>

#include "Common.h"
#include "LIDARLite.h"

namespace {
LIDARLite gLidar;

constexpr uint8_t kLidarAddress = 0x62;

bool ProbeLidar() {
    Wire.beginTransmission(kLidarAddress);
    return Wire.endTransmission() == 0;
}
}

bool Lidar::Initialize() {
    data_ = {};
    diagnostics_ = {};
    initialized_ = false;
    consecutiveFailures_ = 0;
    data_.status = Common::LidarStatus::Uninitialized;
    data_.valid = false;

    // The wrapped library owns the device-specific register sequencing.
    Wire.begin();
    Wire.setClock(Common::I2C_BAUDRATE);

    // A missing ACK gives us a real absent-hardware failure before we commit to
    // the library init path.
    if (!ProbeLidar()) {
        ++consecutiveFailures_;
        data_.failureCount = consecutiveFailures_;
        data_.status = Common::LidarStatus::NoAck;
        ++diagnostics_.initFailureCount;
        return false;
    }

    gLidar.begin(0, true);

    // Probe once more after library setup so startup failure stays explicit.
    if (!ProbeLidar()) {
        ++consecutiveFailures_;
        data_.failureCount = consecutiveFailures_;
        data_.status = Common::LidarStatus::NoAck;
        ++diagnostics_.initFailureCount;
        return false;
    }

    initialized_ = true;
    data_.failureCount = 0;
    data_.status = Common::LidarStatus::Ok;
    return true;
}

Common::LidarData Lidar::ReadData() {
    if (!initialized_) {
        data_.valid = false;
        data_.status = Common::LidarStatus::Uninitialized;
        return data_;
    }

    if (!ProbeLidar()) {
        ++consecutiveFailures_;
        data_.failureCount = consecutiveFailures_;
        data_.valid = false;
        data_.status = Common::LidarStatus::NoAck;
        ++diagnostics_.readFailureCount;
        return data_;
    }

    // Library returns range in centimeters for the default API.
    const int distanceCm = gLidar.distance();
    if (distanceCm <= 0) {
        ++consecutiveFailures_;
        data_.failureCount = consecutiveFailures_;
        data_.valid = false;
        data_.status = Common::LidarStatus::InvalidRange;
        ++diagnostics_.readFailureCount;
        return data_;
    }

    consecutiveFailures_ = 0;
    data_.distanceMeters = static_cast<float>(distanceCm) * 0.01f;
    data_.failureCount = 0;
    data_.status = Common::LidarStatus::Ok;
    data_.valid = true;
    return data_;
}

Lidar::Diagnostics Lidar::GetDiagnostics() const {
    return diagnostics_;
}

void Lidar::ClearDiagnostics() {
    diagnostics_ = {};
}
