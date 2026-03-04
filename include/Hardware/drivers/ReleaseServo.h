#ifndef HARDWARE_RELEASE_SERVO_H
#define HARDWARE_RELEASE_SERVO_H

#include <cstdint>

#include <Servo.h>

#include "../interfaces/ServoInterface.h"

class ReleaseServo : public IServo {
public:
    // Default constructor is required so arrays of actuators can be declared globally.
    ReleaseServo() = default;

    // Assigns/updates the PWM pin used by this servo.
    // Must be called before Initialize() when using the default constructor.
    void ConfigurePin(uint8_t pin);

    bool Initialize() override;
    void SetAngle(float degrees) override;

private:
    // Commanded angle range used by this driver API.
    // We expose standard hobby-servo convention: 0 deg = one endpoint, 180 deg = opposite endpoint.
    static constexpr float kMinAngleDeg = 0.0f;
    static constexpr float kMaxAngleDeg = 180.0f;
    // Boot-up position defaults to neutral to avoid slamming hard end-stops at startup.
    // Mission logic can immediately command a different lock/release angle if needed.
    static constexpr float kStartupAngleDeg = 90.0f;

    // SG90 hard pulse limits and nominal control window from the provided datasheet bundle.
    // - 500..2400 us appears as max electrical range in several SG90 sheets.
    // - 1000..2000 us corresponds to the common 180 deg mechanical command range.
    static constexpr int kPulseHardMinUs = 500;
    static constexpr int kPulseHardMaxUs = 2400;
    static constexpr int kPulseCmdMinUs = 1000;
    static constexpr int kPulseCmdMaxUs = 2000;

    // Special value meaning no valid output pin has been assigned.
    static constexpr uint8_t kInvalidPin = 0xFF;

    static float Clamp(float value, float minValue, float maxValue);
    int AngleToPulseMicros(float degrees) const;

    Servo servo_{};
    uint8_t pin_ = kInvalidPin;
    // Caches the last requested command so startup can apply it and telemetry/debug can inspect it.
    float lastCommandedAngleDeg_ = kStartupAngleDeg;
    bool initialized_ = false;
};

#endif  // HARDWARE_RELEASE_SERVO_H
