#ifndef HARDWARE_LIDAR_DRIVER_H
#define HARDWARE_LIDAR_DRIVER_H

#include "interfaces/LidarInterface.h"

class Lidar : public ILidar {
public:
    static constexpr const char* kSensorName = "Generic Lidar";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::LidarData ReadData() override;
};

#endif  // HARDWARE_LIDAR_DRIVER_H
