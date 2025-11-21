#include "Communications/Commlink.h"
#include <Arduino.h>

void Commlink::Initialize(){
    Serial.begin(SERIAL_BAUDRATE);
    
}