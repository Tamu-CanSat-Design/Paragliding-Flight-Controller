#include "Hardware/drivers/AudioBeacon.h"

#include <Arduino.h>

#include "Common.h"

bool AudioBeacon::Initialize() {
    diagnostics_ = {};
    initialized_ = false;
    pinMode(Common::BUZZER_PIN, OUTPUT);
    digitalWrite(Common::BUZZER_PIN, LOW);
    initialized_ = true;
    return true;
}

// CPI-3116-3-100T is active/self-oscillating — digitalWrite only, no tone().
void AudioBeacon::BuzzerOn() {
    if (!initialized_) { return; }
    digitalWrite(Common::BUZZER_PIN, HIGH);
}

void AudioBeacon::BuzzerOff() {
    if (!initialized_) { return; }
    digitalWrite(Common::BUZZER_PIN, LOW);
}

AudioBeacon::Diagnostics AudioBeacon::GetDiagnostics() const {
    return diagnostics_;
}

void AudioBeacon::ClearDiagnostics() {
    diagnostics_ = {};
}
