#include <Arduino.h> 
#include <Hardware/drivers/TeseoGPS.h>


TeseoGPS gps;

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

    Serial.print("GPS: lat=");
    Serial.print(sample.latitude, 6);
    Serial.print(" lon=");
    Serial.print(sample.longitude, 6);
    Serial.print(" alt=");
    Serial.print(sample.altitude, 2);
    Serial.print(" sats=");
    Serial.print(sample.satellites);
    Serial.println();

}