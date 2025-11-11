#include <deque>



enum StatusCheck{
    HEALTHY=0,
    WARN=1,
    FAULT=2,
};

struct SensorWindow{
    std::deque<float> SlidingWindow;
    unsigned int ExpectedSize;
    float Sum;
    float mean;
    
};

class HealthMonitor{

public:
    HealthMonitor(int _expectedSize);
    void Initialize();
    void Update(float value);
    void MovingAverage();
    void MahalanobisDistance();

private:
    SensorWindow buffer;

};