#ifndef HARDWARE_SIMULATED_LIDAR_H
#define HARDWARE_SIMULATED_LIDAR_H

#include "interfaces/Lidar.h"

class SimulatedLidar : public ILidar {
public:
    static constexpr const char* kSensorName = "Simulated Lidar";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    LidarData ReadData() override;

private:
    LidarData data_{};
};

#endif  // HARDWARE_SIMULATED_LIDAR_H
