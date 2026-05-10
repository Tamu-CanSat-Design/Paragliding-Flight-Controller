#include <Arduino.h> 
#include <Hardware/drivers/TeseoGPS.h>


TeseoGPS gps;

namespace {

void PrintBool(const bool value) {
    Serial.print(value ? "Y" : "N");
}

void PrintGpsSample(const Common::GPSData& sample) {
    Serial.print("GPS: time=");
    Serial.print(sample.time, 2);
    Serial.print(" lat=");
    Serial.print(sample.latitude, 6);
    Serial.print(" lon=");
    Serial.print(sample.longitude, 6);
    Serial.print(" alt=");
    Serial.print(sample.altitude, 2);
    Serial.print(" sats=");
    Serial.print(sample.satellites);
    Serial.print(" speed_mps=");
    Serial.print(sample.groundSpeedMetersPerSecond, 3);
    Serial.print(" speed_valid=");
    PrintBool(sample.groundSpeedValid);
    Serial.print(" cog_deg=");
    Serial.print(sample.courseOverGroundDegrees, 2);
    Serial.print(" cog_valid=");
    PrintBool(sample.courseOverGroundValid);
    Serial.println();
}

void PrintGpsDiagnostics(const TeseoGPS::Diagnostics& diagnostics) {
    Serial.print("GPS diag: initFail=");
    Serial.print(diagnostics.initFailureCount);
    Serial.print(" readFail=");
    Serial.print(diagnostics.readFailureCount);
    Serial.print(" stale=");
    Serial.print(diagnostics.staleReturnCount);
    Serial.print(" idle=");
    Serial.print(diagnostics.idleByteCount);
    Serial.print(" badChecksum=");
    Serial.print(diagnostics.badChecksumCount);
    Serial.print(" unknown=");
    Serial.print(diagnostics.unknownSentenceCount);
    Serial.print(" invalidSpeed=");
    Serial.print(diagnostics.invalidGroundSpeedCount);
    Serial.print(" invalidCog=");
    Serial.print(diagnostics.invalidCourseOverGroundCount);
    Serial.println();
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    

    const bool gpsOk = gps.Initialize();
    Serial.print("GPS init: ");
    Serial.println(gpsOk ? "OK" : "FAIL");
    //suspend if failed
    if (!gpsOk) {
        while (true) {
            delay(1000);
        }
    }

   
   
}

void loop() {
    
    


    const Common::GPSData sample = gps.ReadData();
    const TeseoGPS::Diagnostics diagnostics = gps.GetDiagnostics();

    PrintGpsSample(sample);
    PrintGpsDiagnostics(diagnostics);
    Serial.println();
    delay(1000);

}
