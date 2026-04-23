#include <Arduino.h>

#include "Common.h"
#include "Hardware/drivers/VoltageSensor.h"

namespace {
VoltageSensor gVoltageSensor;
constexpr unsigned long kPrintPeriodMs = 250;
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("voltage_bench: starting");

    const bool ok = gVoltageSensor.Initialize();
    Serial.print("voltage_bench: init=");
    Serial.println(ok ? "OK" : "FAIL");
}

void loop() {
    static unsigned long lastPrintMs = 0;
    const unsigned long nowMs = millis();
    if ((nowMs - lastPrintMs) < kPrintPeriodMs) {
        return;
    }
    lastPrintMs = nowMs;

    const Common::VoltageData data = gVoltageSensor.ReadData();
    const VoltageSensor::Diagnostics diag = gVoltageSensor.GetDiagnostics();

    // Units: voltage_mV, current_mA, power_mW (INA260 LSB scaling applied by driver).
    Serial.print("voltage_bench: voltage_mV=");
    Serial.print(data.voltage, 2);
    Serial.print(" current_mA=");
    Serial.print(data.current, 2);
    Serial.print(" power_mW=");
    Serial.print(data.power, 2);
    Serial.print(" diag={initFail=");
    Serial.print(diag.initFailureCount);
    Serial.print(",readFail=");
    Serial.print(diag.readFailureCount);
    Serial.print(",stale=");
    Serial.print(diag.staleReturnCount);
    Serial.println("}");
}
