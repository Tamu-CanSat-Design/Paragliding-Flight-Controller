#include "Hardware/drivers/Mg90sServo.h"

#include <math.h>

namespace {

constexpr int kMinPulseUs = 1000;
constexpr int kMaxPulseUs = 2000;
constexpr float kMinAngleDeg = 0.0f;
constexpr float kMaxAngleDeg = 180.0f;
constexpr float kStartupAngleDeg = 90.0f;

bool IsFinite(float value) {
    return isfinite(value);
}

float ClampAngle(float degrees) {
    if (degrees < kMinAngleDeg) {
        return kMinAngleDeg;
    }
    if (degrees > kMaxAngleDeg) {
        return kMaxAngleDeg;
    }
    return degrees;
}

}  // namespace

Mg90sServo::Mg90sServo(uint8_t pin)
    : pin_(pin) {}

bool Mg90sServo::Initialize() {
    servo_.attach(pin_, kMinPulseUs, kMaxPulseUs);
    if (!servo_.attached()) {
        initialized_ = false;
        return false;
    }

    initialized_ = true;
    SetAngle(kStartupAngleDeg);
    return true;
}

void Mg90sServo::SetAngle(float degrees) {
    if (!initialized_ || !IsFinite(degrees)) {
        return;
    }

    if (!servo_.attached()) {
        initialized_ = false;
        return;
    }

    const float commandedDegrees = ClampAngle(degrees);
    servo_.write(static_cast<int>(commandedDegrees + 0.5f));
}
