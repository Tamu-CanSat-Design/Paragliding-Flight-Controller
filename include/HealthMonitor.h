#ifndef HARDWARE_HEALTHMONITOR_H
#define HARDWARE_HEALTHMONITOR_H

#include <deque>



enum StatusCheck{
    HEALTHY=0,
    WARN=1,
    FAULT=2,
};

struct SensorWindow{
    std::deque<float> SlidingWindow;
    unsigned int ExpectedSize;
    float Sum = 0;
    float mean;
    
};

class HealthMonitor{

public:
    HealthMonitor(int _expectedSize);
    void Initialize();
    void UpdateWindow(float value);
    void MovingAverage();

private:
    SensorWindow buffer;

};

#endif // HARDWARE_HEALTHMONITOR_H