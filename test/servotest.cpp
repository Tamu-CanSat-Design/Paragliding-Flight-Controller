#include <Arduino.h>
#include "Hardware/drivers/SteeringServo.h"

// =============================================================================
// SteeringServo hardware bench test — goBILDA 2000 Series (25-2 Torque)
//
// SAFETY CHECKLIST before powering up:
//   [ ] Servo powered from external BEC/supply, NOT Arduino 5V pin.
//   [ ] Supply ground connected to Arduino GND (common ground).
//   [ ] Mechanical linkage is clear and free to move.
//   [ ] FULL_SWEEP_ENABLED is false on first run (narrow range test only).
//
// Upload this sketch, open Serial Monitor at 115200 baud, and follow prompts.
// =============================================================================

// --- Configuration -----------------------------------------------------------

// Set this to the Teensy pin connected to the servo signal wire.
static constexpr uint8_t kServoPin = 15;

// Narrow mechanical limits for first-run safety.
// Validate that these endpoints don't bind before widening toward 0 / 2500 µs.
static constexpr int kSafeMinPulse    = 1000;   // ~66.7 deg
static constexpr int kSafeCenterPulse = 1500;   // 150 deg
static constexpr int kSafeMaxPulse    = 2000;   // ~233.3 deg

// Set to true ONLY after the safe range test passes without binding.
static constexpr bool kFullSweepEnabled = false;

// -----------------------------------------------------------------------------

namespace {

SteeringServo::Config makeConfig() {
    SteeringServo::Config c;
    c.pin          = kServoPin;
    c.debugEnabled = true;
    c.clampCommands = true;

    if (kFullSweepEnabled) {
        c.minPulseUs    = 500;
        c.centerPulseUs = 1500;
        c.maxPulseUs    = 2500;
        c.minAngleDeg   = 0.0f;
        c.centerAngleDeg = 150.0f;
        c.maxAngleDeg   = 300.0f;
    } else {
        c.minPulseUs    = kSafeMinPulse;
        c.centerPulseUs = kSafeCenterPulse;
        c.maxPulseUs    = kSafeMaxPulse;
        c.minAngleDeg   = static_cast<float>(kSafeMinPulse  - 500) * 0.150f;
        c.centerAngleDeg = static_cast<float>(kSafeCenterPulse - 500) * 0.150f;
        c.maxAngleDeg   = static_cast<float>(kSafeMaxPulse  - 500) * 0.150f;
    }
    return c;
}

SteeringServo gServo{makeConfig()};

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n========================================"));
    Serial.println(F("  SteeringServo Bench Test"));
    Serial.println(F("  goBILDA 2000 Series 25-2 Torque"));
    Serial.println(F("========================================"));

    if (!kFullSweepEnabled) {
        Serial.println(F("[INFO] Running in SAFE RANGE mode."));
        Serial.print(F("[INFO] Pulse limits: "));
        Serial.print(kSafeMinPulse); Serial.print(F(" - ")); Serial.print(kSafeMaxPulse);
        Serial.println(F(" us"));
        Serial.println(F("[INFO] Set kFullSweepEnabled=true after validating this range."));
    } else {
        Serial.println(F("[INFO] FULL SWEEP enabled (0-300 deg). Ensure linkage is clear."));
    }

    const bool ok = gServo.Initialize();
    Serial.print(F("\nServo init: "));
    Serial.println(ok ? F("OK") : F("FAIL"));
    if (!ok) {
        Serial.println(F("[ERROR] Initialize() failed. Check pin number and wiring."));
        while (true) { delay(1000); }
    }

    // Print configuration so it can be verified before motion begins.
    gServo.printConfig(Serial);

    delay(1000);
    gServo.runDiagnostics(Serial);
}

void loop() {
    // Interactive normalized sweep so the servo can be tested manually after diagnostics.
    static unsigned long lastMs = 0;
    static int step = 0;
    const unsigned long kIntervalMs = 1500;

    if (millis() - lastMs < kIntervalMs) return;
    lastMs = millis();

    const float values[] = { 0.0f, 0.5f, 1.0f, 0.5f, 0.0f, -0.5f, -1.0f, -0.5f };
    const int   n        = static_cast<int>(sizeof(values) / sizeof(values[0]));

    const float v = values[step % n];
    gServo.setNormalized(v);

    Serial.print(F("loop norm="));
    Serial.print(v, 1);
    Serial.print(F("  -> "));
    Serial.print(gServo.getLastPulseUs());
    Serial.print(F(" us / "));
    Serial.print(gServo.getLastAngleDeg(), 1);
    Serial.println(F(" deg"));

    step++;
}
