#include "Hardware/drivers/Mg90sServo.h"

#include <math.h>

namespace {

constexpr int kMinPulseUs = 1000;
constexpr int kMaxPulseUs = 2000;
constexpr int kStopPulseUs = 1500;
constexpr float kMinCommandDeg = -90.0f;
constexpr float kMaxCommandDeg = 90.0f;

bool IsFinite(float value) {
    return isfinite(value);
}

float ClampCommand(float degrees) {
    if (degrees < kMinCommandDeg) {
        return kMinCommandDeg;
    }
    if (degrees > kMaxCommandDeg) {
        return kMaxCommandDeg;
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
    servo_.writeMicroseconds(kStopPulseUs);
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

    const float commandedDegrees = ClampCommand(degrees);
    const int pulseWidth = static_cast<int>(
        kStopPulseUs + (commandedDegrees / kMaxCommandDeg) * 500.0f);
    servo_.writeMicroseconds(pulseWidth);
}
