#ifndef HARDWARE_RELEASE_SERVO_H
#define HARDWARE_RELEASE_SERVO_H

#include "../interfaces/ServoInterface.h"
#include <Servo.h>
#include <stdint.h>

enum class ServoStatus : uint8_t {
    Uninitialized = 0,
    Ok            = 1,
    Detached      = 2,  // servo_.attached() returned false after Initialize()
};

struct ServoDiagnostics {
    ServoStatus status;
    float       commandedDeg;    // last angle written to the servo
    bool        isSlewing;       // SlewTo currently in progress
    uint32_t    clampCount;      // times SetAngle received an out-of-range request
    uint32_t    slewCount;       // completed SlewTo calls
    uint32_t    slewStepCount;   // total PWM steps taken across all slews
};

class ReleaseServo : public IServo {
public:
    explicit ReleaseServo(uint8_t pin)
        : pin_(pin), currentDeg_(kLockedDeg), status_(ServoStatus::Uninitialized),
          isSlewing_(false), clampCount_(0), slewCount_(0), slewStepCount_(0) {}

    bool Initialize() override;

    // Instant move to angle (0–180°).
    void SetAngle(float degrees) override;

    // Slew to angle at a controlled rate. Blocks the calling task.
    // degsPerSec clamped to [kMinDegsPerSec, kMaxDegsPerSec].
    void SlewTo(float targetDeg, float degsPerSec);

    // Instant locked / released positions.
    void Lock();
    void Release();

    // Speed-controlled locked / released positions.
    void LockSlewed(float degsPerSec);
    void ReleaseSlewed(float degsPerSec);

    // Diagnostics.
    ServoDiagnostics GetDiagnostics() const;
    void             ResetDiagnostics();

private:
    uint8_t     pin_;
    Servo       servo_;
    float       currentDeg_;
    ServoStatus status_;
    bool        isSlewing_;
    uint32_t    clampCount_;
    uint32_t    slewCount_;
    uint32_t    slewStepCount_;

    // MG90S physical pulse range (µs)
    static constexpr int   kMinPulseUs     = 500;
    static constexpr int   kMaxPulseUs     = 2400;

    // Application-defined locked / released positions (degrees)
    static constexpr float kLockedDeg      = 0.0f;
    static constexpr float kReleasedDeg    = 90.0f;

    // Slew rate limits (degrees/second)
    static constexpr float kMinDegsPerSec  = 1.0f;
    static constexpr float kMaxDegsPerSec  = 300.0f;

    // Step interval for SlewTo (ms)
    static constexpr int   kStepIntervalMs = 20;
};

#endif  // HARDWARE_RELEASE_SERVO_H
