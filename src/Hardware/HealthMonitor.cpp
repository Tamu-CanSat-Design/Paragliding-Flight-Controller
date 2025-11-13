#include "Hardware/HealthMonitor.h"
// #define SLIDING_WINDOW_SIZE 5


void HealthMonitor::Update(float value){
    if(buffer.SlidingWindow.size() == buffer.ExpectedSize)
    {
        buffer.Sum -= buffer.SlidingWindow.front();
        buffer.SlidingWindow.pop_front();
        buffer.SlidingWindow.push_back(value);
    }

    buffer.Sum += value;
    buffer.SlidingWindow.push_back(value);

    
}