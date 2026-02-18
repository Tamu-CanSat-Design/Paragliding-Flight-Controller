#include "Commlink.h"
#include <Arduino.h>
#include <string.h>

void Commlink::Initialize(){
/* Initialize Ground Link */
    XBEE_SERIAL.begin(XBEE_BAUDRATE);
}

void Commlink::WriteGroundRadio(const char* data){
/* If data exists, write to XBEE and flush buffer */
    if (!data){
        return;
    }
    XBEE_SERIAL.println(data);
    XBEE_SERIAL.flush();
}
void Commlink::ReadGroundRadio(char* buffer, int length){
/* If data exists, parse */
    if (length<=0){
        return;
    }
    std::string text;
}
void Commlink::ParseCommand(const char* command){
/* Parse possible commands, add to buffer? */

}