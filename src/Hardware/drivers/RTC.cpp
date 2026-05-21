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

char* RTC::GetTime(){
    time_t currentTime = Teensy3Clock.get();
    

    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
             hour(currentTime), minute(currentTime), second(currentTime));
    return timeStr;
}

