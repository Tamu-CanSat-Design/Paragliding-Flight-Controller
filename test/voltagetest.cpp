#include <Arduino.h>
#include "Common.h"
#include "Hardware/drivers/VoltageSensor.h"


VoltageSensor voltageSensor;

void setup() {
    Serial.begin(115200);
    delay(1000);

    const bool voltageOk = voltageSensor.Initialize();
    Serial.print("Voltage sensor init: ");
    Serial.println(voltageOk ? "OK" : "FAIL");
    //suspend if failed
    if (!voltageOk) {
        while (true) {
            delay(1000);
        }
    }
   
}

void loop() {
    const Common::VoltageData sample = voltageSensor.ReadData();

    Serial.print("Voltage: ");
    Serial.print(sample.voltage, 2);
    Serial.print(" mV, Current: ");
    Serial.print(sample.current, 2);
    Serial.print(" mA, Power: ");
    Serial.print(sample.power, 2);
    Serial.println(" mW");
}