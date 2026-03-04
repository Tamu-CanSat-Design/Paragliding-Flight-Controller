#include "Hardware/drivers/ReleaseServo.h"

namespace {
// The Servo library expects integer microsecond commands.
constexpr float kMicrosRoundBias = 0.5f;
}  // namespace

void ReleaseServo::ConfigurePin(uint8_t pin) {
    // If the driver is already active, detach first so timer resources are released
    // before changing to a new output pin.
    if (initialized_ && servo_.attached()) {
        servo_.detach();
        initialized_ = false;
    }
    pin_ = pin;
}

bool ReleaseServo::Initialize() {
    // Clear state first so failed init never leaves us marked "ready".
    initialized_ = false;

    // Pin must be configured before we can attach the servo output.
    if (pin_ == kInvalidPin) {
        return false;
    }

    // Attach configures the PWM generator for this pin.
    // Use the datasheet hard range here so library-side clamping is safe for any command.
    servo_.attach(pin_, kPulseHardMinUs, kPulseHardMaxUs);
    if (!servo_.attached()) {
        return false;
    }

    // Apply the currently cached command immediately after attach.
    // By default this is kStartupAngleDeg, but pre-init SetAngle() calls are preserved.
    servo_.writeMicroseconds(AngleToPulseMicros(lastCommandedAngleDeg_));

    initialized_ = true;
    return true;
}

void ReleaseServo::SetAngle(float degrees) {
    // Always sanitize input so invalid commands never exceed servo travel limits.
    lastCommandedAngleDeg_ = Clamp(degrees, kMinAngleDeg, kMaxAngleDeg);

    // If not initialized yet, keep the command cached and apply it during/after init.
    if (!initialized_) {
        return;
    }

    // Convert to pulse-width command and send to the servo.
    // SG90 uses a ~20 ms frame with pulse width encoding target angle.
    servo_.writeMicroseconds(AngleToPulseMicros(lastCommandedAngleDeg_));
}

float ReleaseServo::Clamp(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

int ReleaseServo::AngleToPulseMicros(float degrees) const {
    // Normalize command into [0, 1] so it can be scaled into pulse width space.
    const float normalized = (degrees - kMinAngleDeg) / (kMaxAngleDeg - kMinAngleDeg);
    // Map 0..180 deg into the nominal SG90 control window (1000..2000 us).
    const float pulseRange = static_cast<float>(kPulseCmdMaxUs - kPulseCmdMinUs);
    const float pulseUs = static_cast<float>(kPulseCmdMinUs) + (normalized * pulseRange);
    return static_cast<int>(pulseUs + kMicrosRoundBias);
}
