#include <Arduino.h> 
#include <Hardware/drivers/TeseoGPS.h>


TeseoGPS gps;

namespace {

void PrintBool(const bool value) {
    Serial.print(value ? "Y" : "N");
}

void PrintGpsSample(const Common::GPSData& sample) {
    Serial.print("GPS: time=");
    Serial.print(sample.time);
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
    Serial.print("GPS diag core: initFail=");
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

    Serial.print("GPS diag i2c: attempts=");
    Serial.print(diagnostics.i2cReadAttemptCount);
    Serial.print(" success=");
    Serial.print(diagnostics.i2cReadSuccessCount);
    Serial.print(" bytesReq=");
    Serial.print(diagnostics.i2cBytesRequestedCount);
    Serial.print(" bytesRx=");
    Serial.print(diagnostics.i2cBytesReceivedCount);
    Serial.print(" last=0x");
    Serial.print(diagnostics.lastI2cByte, HEX);
    Serial.print(" lastData=0x");
    Serial.print(diagnostics.lastNonIdleByte, HEX);
    Serial.println();

    Serial.print("GPS diag parser: dataBytes=");
    Serial.print(diagnostics.nonIdleByteCount);
    Serial.print(" clears=");
    Serial.print(diagnostics.parserClearCount);
    Serial.print(" complete=");
    Serial.print(diagnostics.sentenceCompleteCount);
    Serial.print(" empty=");
    Serial.print(diagnostics.emptySentenceCount);
    Serial.print(" lastMsg=");
    Serial.print(diagnostics.lastMessageId);
    Serial.println();

    Serial.print("GPS diag sentences: gga=");
    Serial.print(diagnostics.ggaSentenceCount);
    Serial.print(" ggaValid=");
    Serial.print(diagnostics.ggaValidCount);
    Serial.print(" ggaInvalid=");
    Serial.print(diagnostics.ggaInvalidFixCount);
    Serial.print(" rmc=");
    Serial.print(diagnostics.rmcSentenceCount);
    Serial.print(" rmcValid=");
    Serial.print(diagnostics.rmcValidCount);
    Serial.print(" rmcInvalid=");
    Serial.print(diagnostics.rmcInvalidFixCount);
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
    delay(100);

}
