#include "Hardware/drivers/RTC.h"


RTC rtc;

void setup(){
    Serial.begin(115200);
    delay(1000);


    if (!rtc.Initialize()) {
        Serial.println("RTC initialization failed!");
        while (true) {
            delay(1000);
        }
    }

    time_t currentTime = rtc.GetTime().unix;
    Serial.print("Current RTC time (unix): ");
    Serial.println(currentTime);

    // Set RTC to a specific time (e.g., Jan 1, 2024, 00:00:00)
    time_t newTime = 1704067200; // Unix timestamp for Jan 1, 2024
    if (!rtc.SetTime(newTime)) {
        Serial.println("RTC set time failed!");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("RTC time set successfully.");
}



void loop(){
    Common::TimeData timeData = rtc.GetTime();
    Serial.print("RTC Time: ");
    Serial.print(timeData.yr); Serial.print("-");
    Serial.print(timeData.mnth); Serial.print("-");
    Serial.print(timeData.dy); Serial.print(" ");
    Serial.print(timeData.hr); Serial.print(":");
    Serial.print(timeData.min); Serial.print(":");
    Serial.print(timeData.sec); Serial.print(" (unix: ");
    Serial.print(timeData.unix);
    Serial.println(")");
    delay(1000);
}