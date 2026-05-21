

#ifndef RTCINTERFACE_H
#define RTCINTERFACE_H
#include "Common.h"
#include <TimeLib.h>
class IRTC{
public:
    virtual ~IRTC() = default;
    virtual bool Initialize() = 0;
    virtual bool SetTime(time_t unix) = 0; //H:M:S
    virtual char* GetTime() = 0; // Returns time in milliseconds since epoch or since power on, depending on implementation
};


#endif // RTCINTERFACE_H
