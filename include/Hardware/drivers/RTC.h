#ifndef HARDWARE_RTC_H
#define HARDWARE_RTC_H
#include "../interfaces/RTCInterface.h"
#include <TimeLib.h>
#include "Common.h"
class RTC : public IRTC {

public:
    bool Initialize() override;
    bool SetTime(time_t unix) override;
    char* GetTime() override;

private:
    char timeStr[Common::TIME_STRING_LENGTH] = {};
};



#endif // HARDWARE_RTC_H