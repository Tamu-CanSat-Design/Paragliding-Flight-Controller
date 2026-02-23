#ifndef HARDWARE_LIDAR_H
#define HARDWARE_LIDAR_H

#include "Common.h"

class ILidar {
public:
    virtual ~ILidar() = default;
    virtual bool Initialize() = 0;
    virtual Common::LidarData ReadData() = 0;
};

#endif  // HARDWARE_LIDAR_H
