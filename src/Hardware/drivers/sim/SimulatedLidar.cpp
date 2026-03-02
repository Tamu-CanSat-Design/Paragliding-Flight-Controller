#include "Hardware/interfaces/LidarInterface.h"
class SimulatedLidar : public ILidar {
public:
    bool Initialize();
    Common::LidarData ReadData();
    void setLidarData(const Common::LidarData& data);
private:
    Common::LidarData data_;
};

bool SimulatedLidar::Initialize() {
    data_.distanceMeters = 0.0f;
    return true;
}

Common::LidarData SimulatedLidar::ReadData() {
    return data_;
}
