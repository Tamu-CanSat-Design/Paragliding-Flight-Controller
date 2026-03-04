#include "Hardware/drivers/AudioBeacon.h"

#include <Arduino.h>

void AudioBeacon::ConfigurePin(uint8_t pin) {
    // If already initialized, force output inactive before switching pins.
    if (initialized_ && pin_ != kInvalidPin) {
        ApplyOutput(false);
    }

    initialized_ = false;
    pin_ = pin;
}

void AudioBeacon::ConfigureActiveLevel(bool activeHigh) {
    activeHigh_ = activeHigh;

    // If running, re-apply current requested state with the new polarity immediately.
    if (initialized_) {
        ApplyOutput(requestedOn_);
    }
}

bool AudioBeacon::Initialize() {
    initialized_ = false;

    // Pin must be configured before we can drive the buzzer.
    if (pin_ == kInvalidPin) {
        return false;
    }

    // Datasheet indicates the CPI-3116-3-100T is a continuous-tone indicator;
    // it sounds when driven with DC power. We treat this output as simple ON/OFF.
    // IMPORTANT: datasheet current can reach ~30 mA at rated voltage, so this pin should
    // drive a transistor/MOSFET stage, not source/sink the buzzer directly from the MCU I/O.
    pinMode(pin_, OUTPUT);

    // Always start in a safe silent state first, then apply cached request.
    ApplyOutput(false);
    ApplyOutput(requestedOn_);

    initialized_ = true;
    return true;
}

void AudioBeacon::BuzzerOn() {
    requestedOn_ = true;

    if (!initialized_) {
        return;
    }
    ApplyOutput(true);
}

void AudioBeacon::BuzzerOff() {
    requestedOn_ = false;

    if (!initialized_) {
        return;
    }
    ApplyOutput(false);
}

void AudioBeacon::ApplyOutput(bool on) {
    // Output level depends on whether the external driver stage is active-high or active-low.
    const bool driveHigh = activeHigh_ ? on : !on;
    digitalWrite(pin_, driveHigh ? HIGH : LOW);
}
