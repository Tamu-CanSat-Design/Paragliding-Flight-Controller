// #include <Arduino.h>

// #include "Common.h"
// #include "Hardware/drivers/TeseoGPS.h"

// namespace {
// TeseoGPS gGps;
// constexpr unsigned long kPrintPeriodMs = 1000;
// }  // namespace

// void setup() {
//     Serial.begin(115200);
//     delay(1000);
//     Serial.println("gps_bench: starting");

//     const bool ok = gGps.Initialize();
//     Serial.print("gps_bench: init=");
//     Serial.println(ok ? "OK" : "FAIL");
// }

// void loop() {
//     static unsigned long lastPrintMs = 0;
//     const unsigned long nowMs = millis();
//     if ((nowMs - lastPrintMs) < kPrintPeriodMs) {
//         return;
//     }
//     lastPrintMs = nowMs;

//     const Common::GPSData data = gGps.ReadData();
//     const TeseoGPS::Diagnostics diag = gGps.GetDiagnostics();

//     // lat=0.0, lon=0.0, sats=0 are expected indoors before a fix is acquired.
//     Serial.print("gps_bench: lat=");
//     Serial.print(data.latitude, 6);
//     Serial.print(" lon=");
//     Serial.print(data.longitude, 6);
//     Serial.print(" alt_m=");
//     Serial.print(data.altitude, 2);
//     Serial.print(" sats=");
//     Serial.print(data.satellites);
//     Serial.print(" diag={initFail=");
//     Serial.print(diag.initFailureCount);
//     Serial.print(",readFail=");
//     Serial.print(diag.readFailureCount);
//     Serial.print(",stale=");
//     Serial.print(diag.staleReturnCount);
//     Serial.println("}");
// }
