#include "Common\Common.h"

class HardwareManager{
public:
    void init();
    void read();
    
private:
    Common::telemetry telemetryData; 
    
};