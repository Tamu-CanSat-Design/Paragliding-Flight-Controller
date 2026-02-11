#ifndef HARDWARE_CAMERA_DRIVER_H
#define HARDWARE_CAMERA_DRIVER_H

#include "interfaces/Camera.h"

class Camera : public ICamera {
public:
    static constexpr const char* kSensorName = "Generic Camera";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    bool StartCapture() override;
    bool StopCapture() override;
};

#endif  // HARDWARE_CAMERA_DRIVER_H
