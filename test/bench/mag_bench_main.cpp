// #include <Arduino.h>

// #include "Hardware/drivers/Magnetometer.h"

// namespace {
// Magnetometer gMagnetometer;
// constexpr unsigned long kPrintPeriodMs = 200;

// void PrintMag(const Common::MagData& mag) {
//     Serial.print(mag.magX, 3);
//     Serial.print(',');
//     Serial.print(mag.magY, 3);
//     Serial.print(',');
//     Serial.print(mag.magZ, 3);
// }
// }  // namespace

// void setup() {
//     Serial.begin(115200);
//     delay(1000);
//     Serial.println("mag_bench: starting");

//     const bool ok = gMagnetometer.Initialize();
//     Serial.print("mag_bench: init=");
//     Serial.println(ok ? "OK" : "FAIL");
// }

// void loop() {
//     static unsigned long lastPrintMs = 0;
//     const unsigned long nowMs = millis();
//     if ((nowMs - lastPrintMs) < kPrintPeriodMs) {
//         return;
//     }
//     lastPrintMs = nowMs;

//     const Common::MagData mag = gMagnetometer.ReadData();
//     const Magnetometer::Diagnostics diagnostics = gMagnetometer.GetDiagnostics();

//     Serial.print("mag_bench: xyz=");
//     PrintMag(mag);
//     Serial.print(" diag={initFail=");
//     Serial.print(diagnostics.initFailureCount);
//     Serial.print(",readFail=");
//     Serial.print(diagnostics.readFailureCount);
//     Serial.print(",stale=");
//     Serial.print(diagnostics.staleReturnCount);
//     Serial.println("}");
// }
