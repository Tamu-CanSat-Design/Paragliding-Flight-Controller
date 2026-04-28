#include <Arduino.h>

#include "Common.h"
#include "Hardware/drivers/Camera.h"

namespace {
Camera gCamera;
constexpr unsigned long kPrintPeriodMs = 500;
constexpr unsigned long kTriggerDelayMs = 1500;
constexpr unsigned long kStopDelayMs = 4000;
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("camera_bench: starting");

    const bool ok = gCamera.Initialize();
    Serial.print("camera_bench: init=");
    Serial.println(ok ? "OK" : "FAIL");

    if (ok) {
        // Wait before triggering to match production startup behavior.
        delay(kTriggerDelayMs);
        gCamera.StartCapture();
        Serial.println("camera_bench: start pulse issued on shared trigger");
    }
}

void loop() {
    static unsigned long lastPrintMs = 0;
    static bool stopIssued = false;
    const unsigned long nowMs = millis();
    if (!stopIssued && nowMs >= kStopDelayMs) {
        gCamera.StopCapture();
        stopIssued = true;
        Serial.println("camera_bench: stop pulse issued on shared trigger");
    }
    if ((nowMs - lastPrintMs) < kPrintPeriodMs) {
        return;
    }
    lastPrintMs = nowMs;

    const Camera::Diagnostics diag = gCamera.GetDiagnostics();
    Serial.print("camera_bench: capturing=");
    Serial.print(diag.startCount > diag.stopCount ? "true" : "false");
    Serial.print(" triggerPin=");
    Serial.print(digitalRead(Common::CAMERA_TRIGGER_PIN));
    Serial.print(" diag={startCount=");
    Serial.print(diag.startCount);
    Serial.print(",stopCount=");
    Serial.print(diag.stopCount);
    Serial.println("}");
}
