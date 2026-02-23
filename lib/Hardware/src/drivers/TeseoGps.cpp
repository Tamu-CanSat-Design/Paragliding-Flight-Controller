#include "Common.h"
#include "interfaces/GPSInterface.h"

class TeseoGPS : public IGPS {
public:
    bool Initialize() override;
    Common::GPSData ReadData() override;
};
    




bool TeseoGPS::Initialize() {
    return true;
}

Common::GPSData TeseoGPS::ReadData() {
    return {};
}
