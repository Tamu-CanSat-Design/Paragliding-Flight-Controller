#ifndef HARDWARE_STEERING_SERVO_H
#define HARDWARE_STEERING_SERVO_H

// goBILDA 2000 Series Dual Mode Servo (25-2 Torque)
//
// POWER SAFETY:
//   - Do NOT power from the Arduino/Teensy 5V pin.
//   - Use an external BEC or servo supply rated for stall current (up to 3 A at 7.4 V).
//   - Arduino GND and servo supply GND must share a common ground.
//   - Start testing with narrowed mechanical limits around center before allowing full travel.
//   - Avoid repeated endpoint commands until mechanical range is validated.
//
// PWM:
//   - 50 Hz (20 ms period), digital servo
//   - 500–2500 µs positional range → 0°–300° travel
//   - 0.150°/µs (6.667 µs/°), deadband 4 µs
//   - Increasing pulse = clockwise rotation

#include "../interfaces/ServoInterface.h"
#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>

class SteeringServo : public IServo {
public:
    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------
    struct Config {
        uint8_t pin           = 255;     // Teensy PWM-capable pin; must be set before Initialize()

        // Mechanical limits — default to full servo travel.
        // Narrow these for your specific steering linkage before allowing full travel.
        int   minPulseUs       = 500;    // leftmost commanded pulse (µs)
        int   centerPulseUs    = 1500;   // center / neutral pulse (µs)
        int   maxPulseUs       = 2500;   // rightmost commanded pulse (µs)

        float minAngleDeg      = 0.0f;   // angle at minPulseUs (deg)
        float centerAngleDeg   = 150.0f; // angle at centerPulseUs (deg)
        float maxAngleDeg      = 300.0f; // angle at maxPulseUs (deg)

        // Safety
        bool clampCommands     = true;   // true = clamp out-of-range; false = reject and log
        bool invertDirection   = false;  // true = flip normalized left/right mapping

        // Debug
        bool debugEnabled      = false;  // true = log every command to the serial stream
    };

    // -------------------------------------------------------------------------
    // Diagnostics
    // -------------------------------------------------------------------------
    struct Diagnostics {
        uint32_t commandCount  = 0;   // total successful writes
        uint32_t clampCount    = 0;   // commands silently clamped to range
        uint32_t rejectCount   = 0;   // commands rejected (clampCommands == false)
    };

    // -------------------------------------------------------------------------
    // Construction / lifecycle
    // -------------------------------------------------------------------------
    explicit SteeringServo(const Config& config = Config{});

    // Attach to the configured pin; returns false if pin is invalid.
    bool Initialize() override;

    void Detach();

    // -------------------------------------------------------------------------
    // Primary write API
    // -------------------------------------------------------------------------

    // Writes angle in degrees [minAngleDeg, maxAngleDeg] using the goBILDA formula.
    // Implements IServo::SetAngle — callers can also use writeAngleDegrees directly.
    void SetAngle(float angleDeg) override;
    void writeAngleDegrees(float angleDeg);

    // Writes a raw pulse width [minPulseUs, maxPulseUs].
    void writeMicroseconds(int pulseUs);

    // Normalized command: -1.0 = minPulse side, 0.0 = center, +1.0 = maxPulse side.
    // Respects invertDirection.
    void setNormalized(float value);

    // -------------------------------------------------------------------------
    // Named position helpers
    // -------------------------------------------------------------------------
    void center();
    void fullLeft();   // min pulse if !invertDirection, else max pulse
    void fullRight();  // max pulse if !invertDirection, else min pulse

    // -------------------------------------------------------------------------
    // Conversion utilities
    // -------------------------------------------------------------------------

    // Absolute goBILDA formula: pulse_us = 500 + angleDeg / 0.150
    // Returns value unclamped — callers can clamp themselves if needed.
    int   angleToPulseUs(float angleDeg) const;

    // Inverse: angleDeg = (pulseUs - 500) * 0.150
    float pulseUsToAngle(int pulseUs) const;

    // Piecewise linear through center so asymmetric configs are handled correctly.
    int   normalizedToPulseUs(float value) const;
    float normalizedToAngleDeg(float value) const;

    // -------------------------------------------------------------------------
    // State accessors
    // -------------------------------------------------------------------------
    int   getLastPulseUs()  const { return lastPulseUs_; }
    float getLastAngleDeg() const { return lastAngleDeg_; }
    bool  isAttached()      const { return initialized_ && servo_.attached(); }

    Diagnostics GetDiagnostics() const;
    void        ClearDiagnostics();

    // -------------------------------------------------------------------------
    // Diagnostic / test methods (pass Serial or any Stream)
    // -------------------------------------------------------------------------
    void printConfig(Stream& out) const;
    void printState(Stream& out)  const;

    // Runs all diagnostic tests in sequence.
    void runDiagnostics(Stream& out);

    // Moves from min to max in `steps` steps, then returns to center.
    void sweepTest(Stream& out, int steps = 10, int stepDelayMs = 300);

    // Moves to endpoints with warnings if near absolute hardware limits.
    void endpointTest(Stream& out);

    // Moves to center and reports result.
    void centerTest(Stream& out);

    // Exercises normalized range: -1.0, -0.5, 0.0, +0.5, +1.0, back to center.
    void normalizedTest(Stream& out);

private:
    // Clamp or reject `pulseUs` per config, then write to servo and update state.
    // Returns true if the command was sent.
    bool sendPulse_(int pulseUs);

    // Print a single "[WARN]" line to `out`.
    void warn_(Stream& out, const char* msg) const;

    Config      config_;
    Servo       servo_;
    Diagnostics diagnostics_{};
    int         lastPulseUs_  = 1500;
    float       lastAngleDeg_ = 150.0f;
    bool        initialized_  = false;

    // goBILDA 2000 Series hardware constants
    static constexpr int   kAbsMinPulseUs  = 500;
    static constexpr int   kAbsMaxPulseUs  = 2500;
    static constexpr float kAbsMinAngleDeg = 0.0f;
    static constexpr float kAbsMaxAngleDeg = 300.0f;
    static constexpr float kDegsPerUs      = 0.150f;  // 0.150°/µs per datasheet
    static constexpr float kUsPerDeg       = 1.0f / kDegsPerUs;  // 6.667 µs/°
    static constexpr int   kPwmFreqHz      = 50;
};

#endif  // HARDWARE_STEERING_SERVO_H
