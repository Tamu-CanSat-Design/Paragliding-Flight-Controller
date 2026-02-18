#include "Commlink.h"
#include <Arduino.h>

void Commlink::Initialize(){
/* Initialize Ground Link */
    XBEE_SERIAL.begin(XBEE_BAUDRATE);
}

void WriteGroundRadio(const char* data){
/* If data exists, write to XBEE and flush buffer */
    if (!data){
        return;
    }
    XBEE_SERIAL.print(data);
    XBEE_SERIAL.flush();
}
void ReadGroundRadio(char* buffer, int length){
/* If data exists, parse */

}
void ParseCommand(const char* command){
/* Parse possible commands, add to buffer? */

}