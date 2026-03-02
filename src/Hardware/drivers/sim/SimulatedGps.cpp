#include "Hardware/interfaces/GpsInterface.h"
class SimulatedGps : public IGPS {
public:
    bool Initialize();
    Common::GPSData ReadData();
    void setGPSData(const Common::GPSData& data);
private:
    Common::GPSData data_;
};

bool SimulatedGps::Initialize() {
    data_.latitude = 0.0f;
    data_.longitude = 0.0f;
    data_.altitude = 0.0f;
    data_.time = 0.0f;
    data_.satellites = 0;
    return true;
}

Common::GPSData SimulatedGps::ReadData() {
    return data_;
}
