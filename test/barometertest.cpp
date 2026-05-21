#include <Arduino.h>
#include <Hardware/drivers/BarometerOTS.h>

BarometerOTS baro;

namespace {

void PrintBaroSample(const Common::BarometerData& sample) {
    Serial.print("Baro: pressure=");
    Serial.print(sample.pressure, 4);
    Serial.print(" hPa  temp=");
    Serial.print(sample.temperature, 2);
    Serial.print(" C  alt=");
    Serial.print(sample.altitude, 2);
    Serial.println(" m");
}

void PrintBaroDiagnostics(const BarometerOTS::Diagnostics& diag) {
    Serial.print("Baro diag init: failures=");
    Serial.print(diag.initFailureCount);
    Serial.print(" lastErrCode=");
    Serial.print(diag.lastInitErrorCode);
    Serial.print(" addr=0x");
    Serial.print(diag.foundAddress, HEX);
    Serial.println();

    Serial.print("Baro diag read: total=");
    Serial.print(diag.totalReadCount);
    Serial.print(" fail=");
    Serial.print(diag.readFailureCount);
    Serial.print(" stale=");
    Serial.print(diag.staleReturnCount);
    Serial.print(" consecutive=");
    Serial.print(diag.consecutiveReadFailures);
    Serial.print(" maxConsecutive=");
    Serial.print(diag.maxConsecutiveReadFailures);
    Serial.print(" lastErrCode=");
    Serial.print(diag.lastReadErrorCode);
    Serial.println();
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Barometer test starting");

    const bool ok = baro.Initialize();
    Serial.print("Baro init: ");
    Serial.println(ok ? "OK" : "FAIL");
    PrintBaroDiagnostics(baro.GetDiagnostics());
    Serial.println();

    if (!ok) {
        while (true) {
            delay(1000);
        }
    }
}

void loop() {
    const Common::BarometerData sample = baro.ReadData();
    const BarometerOTS::Diagnostics diag = baro.GetDiagnostics();

    PrintBaroSample(sample);
    PrintBaroDiagnostics(diag);
    Serial.println();
    delay(100);
}
