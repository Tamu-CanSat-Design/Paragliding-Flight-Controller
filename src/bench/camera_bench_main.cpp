// #include <Arduino.h>

// #include "Hardware/drivers/Camera.h"

// namespace {
// Camera gCamera;
// constexpr unsigned long kPrintPeriodMs = 500;
// // Match the production 1-2 s power-up delay before trigger assertion.
// constexpr unsigned long kTriggerDelayMs = 1500;
// }  // namespace

// void setup() {
//     Serial.begin(115200);
//     delay(1000);
//     Serial.println("camera_bench: starting");

//     const bool ok = gCamera.Initialize();
//     Serial.print("camera_bench: init=");
//     Serial.println(ok ? "OK" : "FAIL");

//     if (ok) {
//         // Wait before triggering to match production startup behavior.
//         delay(kTriggerDelayMs);
//         gCamera.StartCapture();
//         Serial.println("camera_bench: trigger=HIGH both ESP32-CAMs should begin recording");
//     }
// }

// void loop() {
//     static unsigned long lastPrintMs = 0;
//     const unsigned long nowMs = millis();
//     if ((nowMs - lastPrintMs) < kPrintPeriodMs) {
//         return;
//     }
//     lastPrintMs = nowMs;

//     // Verify trigger pin with a multimeter: should read HIGH and stay HIGH.
//     const Camera::Diagnostics diag = gCamera.GetDiagnostics();
//     Serial.print("camera_bench: capturing=");
//     Serial.print(diag.startCount > diag.stopCount ? "true" : "false");
//     Serial.print(" diag={startCount=");
//     Serial.print(diag.startCount);
//     Serial.print(",stopCount=");
//     Serial.print(diag.stopCount);
//     Serial.println("}");
// }
