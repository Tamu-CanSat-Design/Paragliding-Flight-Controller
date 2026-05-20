#include "Hardware/drivers/RTC.h"


bool RTC::Initialize() {
    setSyncProvider([]() -> time_t {
        return Teensy3Clock.get();
    });
    setSyncInterval(60);
    return true; // Placeholder return value
}

bool RTC::SetTime(time_t unix){
    Teensy3Clock.set(unix);
    setTime(unix);
    return true;
}

Common::TimeData RTC::GetTime(){
    time_t currentTime = Teensy3Clock.get();
    Common::TimeData timeData;
    timeData.unix = currentTime;
    timeData.year = year(currentTime);
    timeData.month = month(currentTime);
    timeData.day = day(currentTime);
    timeData.hour = hour(currentTime);
    timeData.minute = minute(currentTime);
    timeData.second = second(currentTime);
    return timeData;
}

