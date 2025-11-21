#include "HealthMonitor.h"

class RTKGPS {
public:
    void Initialize();
    float ReadValue();

private:
    float latitude;
    float longitude;
    float altitude;
    float speed;
    int satellites;
    
    HealthMonitor healthMonitor;
};