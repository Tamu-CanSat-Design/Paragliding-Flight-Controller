#include <Arduino.h>
#include "Common.h"
#include "Hardware/drivers/MagnetometerOTS.h"

namespace{
    MagnetometerOTS gMagnetometer;

    
}




void setup() {
    Serial.begin(115200);
    delay(1000);
    Wire1.begin();
    
    const bool magOk = gMagnetometer.Initialize();
    Serial.print("Magnetometer init: ");
    Serial.println(magOk ? "OK" : "FAIL");
    //suspend if failed
    if (!magOk) {
        while (true) {
            delay(1000);
        }
    }

   
   
}

void loop() {
    
    
   

    const Common::MagData sample = gMagnetometer.ReadData();

    Serial.print("magnetic field=");
    Serial.print("mx=");
    Serial.print(sample.magX, 2);
    Serial.print(" my=");
    Serial.print(sample.magY, 2);
    Serial.print(" mz=");
    Serial.print(sample.magZ, 2);
    Serial.println();


}



