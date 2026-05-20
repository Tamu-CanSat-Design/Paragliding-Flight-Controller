#ifndef HARDWARE_RTC_H
#define HARDWARE_RTC_H
#include "../interfaces/RTCInterface.h"
#include <TimeLib.h>
#include "Common.h"
class RTC : public IRTC {

public:
    bool Initialize() override;
    bool SetTime(time_t unix) override;
    Common::TimeData GetTime() override;
};



#endif // HARDWARE_RTC_H