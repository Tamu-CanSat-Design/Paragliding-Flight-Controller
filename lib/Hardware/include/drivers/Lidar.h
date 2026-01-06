#ifndef HARDWARE_LIDAR_DRIVER_H
#define HARDWARE_LIDAR_DRIVER_H

#include "interfaces/Lidar.h"

class Lidar : public ILidar {
public:
    bool Initialize() override;
    LidarData ReadData() override;
};

#endif  // HARDWARE_LIDAR_DRIVER_H
