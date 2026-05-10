// #include <Arduino.h>

// #include "Common.h"
// #include "Hardware/drivers/VoltageSensor.h"

// namespace {
// VoltageSensor gVoltageSensor;
// constexpr unsigned long kPrintPeriodMs = 250;
// }  // namespace

// void setup() {
//     Serial.begin(115200);
//     delay(1000);
//     Serial.println("voltage_bench: starting");

//     const bool ok = gVoltageSensor.Initialize();
//     Serial.print("voltage_bench: init=");
//     Serial.println(ok ? "OK" : "FAIL");
// }

// void loop() {
//     static unsigned long lastPrintMs = 0;
//     const unsigned long nowMs = millis();
//     if ((nowMs - lastPrintMs) < kPrintPeriodMs) {
//         return;
//     }
//     lastPrintMs = nowMs;

//     const Common::VoltageData data = gVoltageSensor.ReadData();
//     const VoltageSensor::Diagnostics diag = gVoltageSensor.GetDiagnostics();

//     // Units: V, A, W (INA260 LSB scaling converted to SI by driver).
//     Serial.print("voltage_bench: voltage_V=");
//     Serial.print(data.voltage, 4);
//     Serial.print(" current_A=");
//     Serial.print(data.current, 4);
//     Serial.print(" power_W=");
//     Serial.print(data.power, 4);
//     Serial.print(" diag={initFail=");
//     Serial.print(diag.initFailureCount);
//     Serial.print(",readFail=");
//     Serial.print(diag.readFailureCount);
//     Serial.print(",stale=");
//     Serial.print(diag.staleReturnCount);
//     Serial.println("}");
// }
