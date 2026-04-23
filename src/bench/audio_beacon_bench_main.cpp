#include <Arduino.h>

#include "Hardware/drivers/AudioBeacon.h"

namespace {
AudioBeacon gAudioBeacon;
constexpr unsigned long kTogglePeriodMs = 1000;
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("audio_beacon_bench: starting");

    const bool ok = gAudioBeacon.Initialize();
    Serial.print("audio_beacon_bench: init=");
    Serial.println(ok ? "OK" : "FAIL");
}

void loop() {
    static unsigned long lastToggleMs = 0;
    static bool buzzerOn = false;

    const unsigned long nowMs = millis();
    if ((nowMs - lastToggleMs) < kTogglePeriodMs) {
        return;
    }
    lastToggleMs = nowMs;

    buzzerOn = !buzzerOn;
    if (buzzerOn) {
        gAudioBeacon.BuzzerOn();
    } else {
        gAudioBeacon.BuzzerOff();
    }

    const AudioBeacon::Diagnostics diag = gAudioBeacon.GetDiagnostics();
    Serial.print("audio_beacon_bench: state=");
    Serial.print(buzzerOn ? "ON" : "OFF");
    Serial.print(" diag={initFail=");
    Serial.print(diag.initFailureCount);
    Serial.println("}");
}
