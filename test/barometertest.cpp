#include <Arduino.h>
#include <Hardware/drivers/BarometerBMP3.h>

BarometerBMP3 baro;

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

void PrintBaroDiagnostics(const BarometerBMP3::Diagnostics& diag) {
    Serial.print("Baro diag init: failures=");
    Serial.print(diag.initFailureCount);
    Serial.print(" lastApi=");
    Serial.print(diag.lastInitApiResult);
    Serial.print(" lastIntf=");
    Serial.print(diag.lastInitInterfaceResult);
    Serial.print(" lastChip=0x");
    Serial.print(diag.lastInitChipId, HEX);
    Serial.print(" configApi=");
    Serial.print(diag.lastConfigApiResult);
    Serial.print(" configIntf=");
    Serial.print(diag.lastConfigInterfaceResult);
    Serial.print(" addr=0x");
    Serial.print(diag.foundAddress, HEX);
    Serial.print(" chip=0x");
    Serial.print(diag.chipId, HEX);
    Serial.println();

    Serial.print("Baro diag probes: primaryApi=");
    Serial.print(diag.primaryInitApiResult);
    Serial.print(" primaryIntf=");
    Serial.print(diag.primaryInitInterfaceResult);
    Serial.print(" primaryChip=0x");
    Serial.print(diag.primaryInitChipId, HEX);
    Serial.print(" secondaryApi=");
    Serial.print(diag.secondaryInitApiResult);
    Serial.print(" secondaryIntf=");
    Serial.print(diag.secondaryInitInterfaceResult);
    Serial.print(" secondaryChip=0x");
    Serial.print(diag.secondaryInitChipId, HEX);
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
    Serial.print(" notReady=");
    Serial.print(diag.notReadyCount);
    Serial.print(" lastApi=");
    Serial.print(diag.lastReadApiResult);
    Serial.print(" lastIntf=");
    Serial.print(diag.lastReadInterfaceResult);
    Serial.println();
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.print("Barometer test starting: ");
    Serial.println(baro.SensorName());

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
    const BarometerBMP3::Diagnostics diag = baro.GetDiagnostics();

    PrintBaroSample(sample);
    PrintBaroDiagnostics(diag);
    Serial.println();
    delay(100);
}
