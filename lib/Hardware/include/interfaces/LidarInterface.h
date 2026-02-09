#ifndef HARDWARE_LIDAR_H
#define HARDWARE_LIDAR_H

struct LidarData {
    float distanceMeters;
};

class ILidar {
public:
    virtual ~ILidar() = default;
    virtual bool Initialize() = 0;
    virtual LidarData ReadData() = 0;
};

#endif  // HARDWARE_LIDAR_H
