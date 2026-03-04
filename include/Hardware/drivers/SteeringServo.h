#ifndef HARDWARE_STEERING_SERVO_H
#define HARDWARE_STEERING_SERVO_H

#include <cstdint>

#include <Servo.h>

#include "../interfaces/ServoInterface.h"

class SteeringServo : public IServo {
public:
    // Default constructor allows global arrays of steering actuators.
    SteeringServo() = default;

    // Assigns/updates the PWM output pin for this steering actuator.
    void ConfigurePin(uint8_t pin);

    bool Initialize() override;
    void SetAngle(float degrees) override;

private:
    // API command range for steering: standard hobby-servo convention in degrees.
    static constexpr float kMinAngleDeg = 0.0f;
    static constexpr float kMaxAngleDeg = 180.0f;
    // Neutral steering command (straight) on startup.
    static constexpr float kStartupAngleDeg = 90.0f;

    // MG90S datasheet pulse points:
    // - 1.0 ms: one endpoint
    // - 1.5 ms: center
    // - 2.0 ms: opposite endpoint
    static constexpr int kPulseMinUs = 1000;
    static constexpr int kPulseMaxUs = 2000;

    static constexpr uint8_t kInvalidPin = 0xFF;

    static float Clamp(float value, float minValue, float maxValue);
    int AngleToPulseMicros(float degrees) const;

    Servo servo_{};
    uint8_t pin_ = kInvalidPin;
    // Last commanded value is cached so commands issued pre-init are not lost.
    float lastCommandedAngleDeg_ = kStartupAngleDeg;
    bool initialized_ = false;
};

#endif  // HARDWARE_STEERING_SERVO_H
