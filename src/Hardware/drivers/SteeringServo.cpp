#include "Hardware/drivers/SteeringServo.h"

// Pulse-width to angle mapping (goBILDA 2000 Series):
//
//   angleDeg  = (pulseUs - 500) * 0.150
//   pulseUs   = 500 + angleDeg / 0.150
//             = 500 + angleDeg * 6.667
//
//   500 µs  →   0°  (hardware minimum)
//  1500 µs  → 150°  (center / neutral)
//  2500 µs  → 300°  (hardware maximum)
//
// All internal writes go through sendPulse_() which enforces configured limits
// and updates lastPulseUs_ / lastAngleDeg_ / diagnostics_.

namespace {
// Clamp helper (avoids pulling in <algorithm>).
template<typename T>
T clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

// -----------------------------------------------------------------------------
// Construction / lifecycle
// -----------------------------------------------------------------------------

SteeringServo::SteeringServo()
    : SteeringServo(Config{}) {}

SteeringServo::SteeringServo(const Config& config)
    : config_(config),
      lastPulseUs_(config.centerPulseUs),
      lastAngleDeg_(config.centerAngleDeg) {}

bool SteeringServo::Initialize() {
    if (config_.pin == 255) return false;
    servo_.attach(config_.pin, kAbsMinPulseUs, kAbsMaxPulseUs);
    if (!servo_.attached()) return false;
    initialized_ = true;
    center();
    return true;
}

void SteeringServo::Detach() {
    servo_.detach();
    initialized_ = false;
}

// -----------------------------------------------------------------------------
// Primary write API
// -----------------------------------------------------------------------------

void SteeringServo::SetAngle(float angleDeg) {
    writeAngleDegrees(angleDeg);
}

void SteeringServo::writeAngleDegrees(float angleDeg) {
    sendPulse_(angleToPulseUs(angleDeg));
}

void SteeringServo::writeMicroseconds(int pulseUs) {
    sendPulse_(pulseUs);
}

void SteeringServo::setNormalized(float value) {
    sendPulse_(normalizedToPulseUs(value));
}

// -----------------------------------------------------------------------------
// Named position helpers
// -----------------------------------------------------------------------------

void SteeringServo::center() {
    sendPulse_(config_.centerPulseUs);
}

void SteeringServo::fullLeft() {
    sendPulse_(config_.invertDirection ? config_.maxPulseUs : config_.minPulseUs);
}

void SteeringServo::fullRight() {
    sendPulse_(config_.invertDirection ? config_.minPulseUs : config_.maxPulseUs);
}

// -----------------------------------------------------------------------------
// Conversion utilities
// -----------------------------------------------------------------------------

int SteeringServo::angleToPulseUs(float angleDeg) const {
    // Absolute goBILDA formula — caller's clamping applied later in sendPulse_().
    return kAbsMinPulseUs + static_cast<int>(angleDeg * kUsPerDeg);
}

float SteeringServo::pulseUsToAngle(int pulseUs) const {
    return static_cast<float>(pulseUs - kAbsMinPulseUs) * kDegsPerUs;
}

int SteeringServo::normalizedToPulseUs(float value) const {
    if (config_.invertDirection) value = -value;
    value = clamp(value, -1.0f, 1.0f);

    // Piecewise linear through center so asymmetric configs stay accurate.
    if (value >= 0.0f) {
        return config_.centerPulseUs
             + static_cast<int>(value * static_cast<float>(config_.maxPulseUs - config_.centerPulseUs));
    } else {
        return config_.centerPulseUs
             + static_cast<int>(value * static_cast<float>(config_.centerPulseUs - config_.minPulseUs));
    }
}

float SteeringServo::normalizedToAngleDeg(float value) const {
    return pulseUsToAngle(normalizedToPulseUs(value));
}

// -----------------------------------------------------------------------------
// State / diagnostics
// -----------------------------------------------------------------------------

SteeringServo::Diagnostics SteeringServo::GetDiagnostics() const {
    return diagnostics_;
}

void SteeringServo::ClearDiagnostics() {
    diagnostics_ = {};
}

// -----------------------------------------------------------------------------
// Diagnostic print methods
// -----------------------------------------------------------------------------

void SteeringServo::printConfig(Stream& out) const {
    out.println(F("=== SteeringServo Config ==="));
    out.print(F("  Servo:        goBILDA 2000 Series (25-2 Torque), 300 deg travel\n"));
    out.print(F("  PWM:          50 Hz, digital, 0.150 deg/us\n"));
    out.print(F("  Pin:          ")); out.println(config_.pin);
    out.print(F("  Pulse range:  ")); out.print(config_.minPulseUs);
    out.print(F(" - ")); out.print(config_.maxPulseUs); out.println(F(" us"));
    out.print(F("  Center pulse: ")); out.print(config_.centerPulseUs); out.println(F(" us"));
    out.print(F("  Angle range:  ")); out.print(config_.minAngleDeg, 1);
    out.print(F(" - ")); out.print(config_.maxAngleDeg, 1); out.println(F(" deg"));
    out.print(F("  Center angle: ")); out.print(config_.centerAngleDeg, 1); out.println(F(" deg"));
    out.print(F("  Clamp:        ")); out.println(config_.clampCommands ? F("ON")  : F("OFF"));
    out.print(F("  Invert dir:   ")); out.println(config_.invertDirection ? F("YES") : F("NO"));
    out.print(F("  Debug log:    ")); out.println(config_.debugEnabled  ? F("ON")  : F("OFF"));
    out.println(F("============================"));
}

void SteeringServo::printState(Stream& out) const {
    out.println(F("=== SteeringServo State ==="));
    out.print(F("  Attached:     ")); out.println(isAttached() ? F("YES") : F("NO"));
    out.print(F("  Last pulse:   ")); out.print(lastPulseUs_); out.println(F(" us"));
    out.print(F("  Last angle:   ")); out.print(lastAngleDeg_, 2); out.println(F(" deg"));
    out.print(F("  Commands:     ")); out.println(diagnostics_.commandCount);
    out.print(F("  Clamped:      ")); out.println(diagnostics_.clampCount);
    out.print(F("  Rejected:     ")); out.println(diagnostics_.rejectCount);
    out.println(F("==========================="));
}

// -----------------------------------------------------------------------------
// Diagnostic test methods
// -----------------------------------------------------------------------------

void SteeringServo::centerTest(Stream& out) {
    out.println(F("\n--- Center Test ---"));
    out.print(F("  Commanding center: ")); out.print(config_.centerPulseUs); out.println(F(" us"));
    center();
    delay(500);
    out.print(F("  Reported angle:    ")); out.print(lastAngleDeg_, 2); out.println(F(" deg"));
    out.println(F("  [OK] Verify servo is physically centered."));
}

void SteeringServo::endpointTest(Stream& out) {
    out.println(F("\n--- Endpoint Test ---"));

    if (config_.minPulseUs <= kAbsMinPulseUs + 10) {
        warn_(out, "minPulseUs is at or near hardware minimum (500 us). Validate linkage before use.");
    }
    if (config_.maxPulseUs >= kAbsMaxPulseUs - 10) {
        warn_(out, "maxPulseUs is at or near hardware maximum (2500 us). Validate linkage before use.");
    }

    out.print(F("  Moving to min: ")); out.print(config_.minPulseUs); out.println(F(" us"));
    center();
    delay(300);
    fullLeft();
    delay(600);

    out.print(F("  Moving to max: ")); out.print(config_.maxPulseUs); out.println(F(" us"));
    fullRight();
    delay(600);

    out.println(F("  Returning to center."));
    center();
    delay(300);
    out.println(F("  [OK] Verify both endpoints move freely without binding."));
}

void SteeringServo::sweepTest(Stream& out, int steps, int stepDelayMs) {
    out.println(F("\n--- Sweep Test ---"));
    if (steps < 2) steps = 2;

    const int range = config_.maxPulseUs - config_.minPulseUs;

    out.print(F("  Sweeping min->max in ")); out.print(steps); out.println(F(" steps."));
    for (int i = 0; i <= steps; i++) {
        const int pulse = config_.minPulseUs + (range * i) / steps;
        out.print(F("    step ")); out.print(i);
        out.print(F(": ")); out.print(pulse); out.print(F(" us -> "));
        out.print(pulseUsToAngle(pulse), 1); out.println(F(" deg"));
        sendPulse_(pulse);
        delay(static_cast<unsigned long>(stepDelayMs));
    }

    out.println(F("  Returning to center."));
    center();
    out.println(F("  [OK] Verify motion was smooth and without binding."));
}

void SteeringServo::normalizedTest(Stream& out) {
    out.println(F("\n--- Normalized Command Test ---"));

    const float values[]    = { -1.0f, -0.5f, 0.0f, 0.5f, 1.0f };
    const int   numValues   = static_cast<int>(sizeof(values) / sizeof(values[0]));

    for (int i = 0; i < numValues; i++) {
        const float  v     = values[i];
        const int    pulse = normalizedToPulseUs(v);
        const float  angle = pulseUsToAngle(pulse);
        out.print(F("  norm="));
        out.print(v, 1);
        out.print(F("  -> ")); out.print(pulse); out.print(F(" us / "));
        out.print(angle, 1); out.println(F(" deg"));
        sendPulse_(pulse);
        delay(400);
    }

    out.println(F("  Returning to center."));
    center();
}

void SteeringServo::runDiagnostics(Stream& out) {
    out.println(F("\n========================================"));
    out.println(F("  SteeringServo Diagnostics"));
    out.println(F("========================================"));

    if (!isAttached()) {
        warn_(out, "Servo is not attached. Call Initialize() first.");
        return;
    }

    printConfig(out);
    printState(out);
    centerTest(out);
    endpointTest(out);
    sweepTest(out);
    normalizedTest(out);

    out.println(F("\n--- Final State ---"));
    printState(out);
    out.println(F("========================================"));
    out.println(F("  Diagnostics complete."));
    out.println(F("========================================\n"));
}

// -----------------------------------------------------------------------------
// Private helpers
// -----------------------------------------------------------------------------

bool SteeringServo::sendPulse_(int pulseUs) {
    const bool tooLow  = pulseUs < config_.minPulseUs;
    const bool tooHigh = pulseUs > config_.maxPulseUs;
    const bool outOfRange = tooLow || tooHigh;

    if (outOfRange) {
        if (config_.clampCommands) {
            pulseUs = clamp(pulseUs, config_.minPulseUs, config_.maxPulseUs);
            ++diagnostics_.clampCount;
            if (config_.debugEnabled) {
                // Lightweight clamp notice — no Stream available here; callers use printState().
            }
        } else {
            ++diagnostics_.rejectCount;
            return false;
        }
    }

    servo_.writeMicroseconds(pulseUs);
    lastPulseUs_  = pulseUs;
    lastAngleDeg_ = pulseUsToAngle(pulseUs);
    ++diagnostics_.commandCount;
    return true;
}

void SteeringServo::warn_(Stream& out, const char* msg) const {
    out.print(F("  [WARN] "));
    out.println(msg);
}
