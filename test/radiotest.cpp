#include <Arduino.h>
#include "Common.h"
#include "Commlink.h"

Commlink commlink;
unsigned long lastSendTime = 0;
void setup(){

    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("Commlink test starting...");

    commlink.Initialize();
    Serial.println("Commlink initialized.");

}


void loop(){
    unsigned long now = millis();




    if (now - lastSendTime >= 1000) {  // Send packet every 1 second

        const char* packet =
        "1030,"
        "17:20:00,"   // timestamp placeholder — use RTC or millis() conversion
        "1,F,LAUNCH_PAD,"
        "0.0,25.5,101.3,12.6,"
        "0.05,0.12,-0.08,"
        "0.15,0.02,0.03,"
        "-0.01,0.1,0.0,0.2,"
        "17:20:00,0.5,"
        "30.6222,-96.3347,12,"
        "CXON,15.2,-8.4,42.3,"
        "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0";


        commlink.WriteGroundRadio(packet);
        lastSendTime = now;
    }



    
}