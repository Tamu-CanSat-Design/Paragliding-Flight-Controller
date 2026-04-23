#ifndef HARDWARE_LIDAR_DRIVER_H
#define HARDWARE_LIDAR_DRIVER_H

#include <cstdint>

#include "../interfaces/LidarInterface.h"

class Lidar : public ILidar {
public:
    struct Diagnostics {
        uint32_t initFailureCount = 0;
        uint32_t readFailureCount = 0;
    };

    static constexpr const char* kSensorName = "LIDARLite";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::LidarData ReadData() override;
    Diagnostics GetDiagnostics() const;
    void ClearDiagnostics();

private:
    Common::LidarData data_{};
    Diagnostics diagnostics_{};
    bool initialized_ = false;
    uint16_t consecutiveFailures_ = 0;
};

#endif  // HARDWARE_LIDAR_DRIVER_H
