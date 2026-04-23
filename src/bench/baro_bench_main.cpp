#include <Arduino.h>

#include "Common.h"
#include "Hardware/drivers/Barometer.h"

namespace {
Barometer gBarometer;
constexpr unsigned long kPrintPeriodMs = 500;
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("baro_bench: starting");

    const bool ok = gBarometer.Initialize();
    Serial.print("baro_bench: init=");
    Serial.println(ok ? "OK" : "FAIL");
}

void loop() {
    static unsigned long lastPrintMs = 0;
    const unsigned long nowMs = millis();
    if ((nowMs - lastPrintMs) < kPrintPeriodMs) {
        return;
    }
    lastPrintMs = nowMs;

    const Common::BarometerData data = gBarometer.ReadData();
    const Barometer::Diagnostics diag = gBarometer.GetDiagnostics();

    // pressure is stored in hPa (Pa * 0.01) by the driver.
    Serial.print("baro_bench: pressure_hPa=");
    Serial.print(data.pressure, 2);
    Serial.print(" temp_C=");
    Serial.print(data.temperature, 2);
    Serial.print(" alt_m=");
    Serial.print(data.altitude, 2);
    Serial.print(" diag={initFail=");
    Serial.print(diag.initFailureCount);
    Serial.print(",readFail=");
    Serial.print(diag.readFailureCount);
    Serial.print(",stale=");
    Serial.print(diag.staleReturnCount);
    Serial.println("}");
}
