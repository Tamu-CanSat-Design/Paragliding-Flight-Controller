#include <Arduino.h>
#include <Wire.h>
#include "Commlink.h"
#include <string>
namespace{
    void clearI2CBus(int sdaPin, int sclPin) {
    pinMode(sclPin, OUTPUT);
    pinMode(sdaPin, INPUT_PULLUP);
    for (int i = 0; i < 9; ++i) {
        digitalWrite(sclPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(sclPin, LOW);
        delayMicroseconds(5);
    }
    // Send a STOP condition.
    pinMode(sdaPin, OUTPUT);
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(5);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(sdaPin, HIGH);
    delayMicroseconds(5);
    Wire.begin();
}


Commlink commlink;

}



void setup() {

    clearI2CBus(18, 19); // Clear the I2C bus by toggling SCL and checking SDA. Adjust pins as needed for your board.
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("Starting I2C scan...");

    Wire.begin();
    Wire.setClock(400000);
    delay(10);

    commlink.Initialize();


    Serial.println("Scanning Wire...");
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("  Found device at 0x%02X\n", addr);
            ++found;
        }
    }
    if (found == 0) Serial.println("  No devices found.");
    while(found == 0){
        commlink.WriteGroundRadio("No devices found");
        delay(1000);
    }
    while(found == 1){
        std::string devices = std::to_string(found);
        const char* pchar = devices.c_str();
        commlink.WriteGroundRadio("pchar");
    }
    Serial.printf("Scan done. %d device(s) found.\n", found);
}

void loop() {}
