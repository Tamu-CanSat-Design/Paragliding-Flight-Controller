#include "Hardware/drivers/SteeringServo.h"

namespace {
// Converts a float pulse width to an integer microsecond command with nearest rounding.
constexpr float kMicrosRoundBias = 0.5f;
}  // namespace

void SteeringServo::ConfigurePin(uint8_t pin) {
    // Detach active output first so timer resources and pin state remain consistent.
    if (initialized_ && servo_.attached()) {
        servo_.detach();
        initialized_ = false;
    }
    pin_ = pin;
}

bool SteeringServo::Initialize() {
    initialized_ = false;

    // Steering servo cannot be initialized until a hardware pin is configured.
    if (pin_ == kInvalidPin) {
        return false;
    }

    // Attach PWM output using MG90S timing limits from datasheet.
    servo_.attach(pin_, kPulseMinUs, kPulseMaxUs);
    if (!servo_.attached()) {
        return false;
    }

    // Apply the currently cached steering command immediately.
    // Default is neutral (kStartupAngleDeg) unless user commanded before init.
    servo_.writeMicroseconds(AngleToPulseMicros(lastCommandedAngleDeg_));

    initialized_ = true;
    return true;
}

void SteeringServo::SetAngle(float degrees) {
    // Clamp to safe mechanical command limits.
    lastCommandedAngleDeg_ = Clamp(degrees, kMinAngleDeg, kMaxAngleDeg);

    // Cache-only behavior before init keeps command ordering deterministic.
    if (!initialized_) {
        return;
    }

    // Send absolute steering command as pulse width.
    servo_.writeMicroseconds(AngleToPulseMicros(lastCommandedAngleDeg_));
}

float SteeringServo::Clamp(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

int SteeringServo::AngleToPulseMicros(float degrees) const {
    // Scale 0..180 deg into 1000..2000 us command range.
    const float normalized = (degrees - kMinAngleDeg) / (kMaxAngleDeg - kMinAngleDeg);
    const float pulseRange = static_cast<float>(kPulseMaxUs - kPulseMinUs);
    const float pulseUs = static_cast<float>(kPulseMinUs) + (normalized * pulseRange);
    return static_cast<int>(pulseUs + kMicrosRoundBias);
}
