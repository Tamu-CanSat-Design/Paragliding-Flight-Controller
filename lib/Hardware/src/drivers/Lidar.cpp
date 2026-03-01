#include "interfaces/LidarInterface.h"

class Lidar : public ILidar {
public:
    bool Initialize();
    Common::LidarData ReadData();
};



bool Lidar::Initialize() {
    return true;
}

Common::LidarData Lidar::ReadData() {
    return {};
}
