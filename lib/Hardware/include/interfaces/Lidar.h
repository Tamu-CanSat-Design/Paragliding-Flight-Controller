#ifndef HARDWARE_LIDAR_H
#define HARDWARE_LIDAR_H

struct LidarData {
    float distanceMeters;
};

class ILidar {
public:
    virtual ~ILidar() = default;
    virtual const char* SensorName() const = 0;
    virtual bool Initialize() = 0;
    virtual LidarData ReadData() = 0;
};

#endif  // HARDWARE_LIDAR_H
