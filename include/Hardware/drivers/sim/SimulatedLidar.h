#ifndef HARDWARE_SIMULATED_LIDAR_H
#define HARDWARE_SIMULATED_LIDAR_H

#include "Hardware/interfaces/LidarInterface.h"

class SimulatedLidar : public ILidar {
public:
    static constexpr const char* kSensorName = "Simulated Lidar";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::LidarData ReadData() override;

private:
    Common::LidarData data_{};
};

#endif  // HARDWARE_SIMULATED_LIDAR_H
