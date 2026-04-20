#ifndef HARDWARE_LIDAR_DRIVER_H
#define HARDWARE_LIDAR_DRIVER_H

#include "../interfaces/LidarInterface.h"

class Lidar : public ILidar {
public:
    static constexpr const char* kSensorName = "LIDARLite";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::LidarData ReadData() override;

private:
    Common::LidarData data_{};
    bool initialized_ = false;
    uint16_t consecutiveFailures_ = 0;
};

#endif  // HARDWARE_LIDAR_DRIVER_H
