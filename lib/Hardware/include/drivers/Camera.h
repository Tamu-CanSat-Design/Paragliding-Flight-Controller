#ifndef HARDWARE_CAMERA_DRIVER_H
#define HARDWARE_CAMERA_DRIVER_H

#include "interfaces/Camera.h"

class Camera : public ICamera {
public:
    bool Initialize() override;
    bool StartCapture() override;
    bool StopCapture() override;
};

#endif  // HARDWARE_CAMERA_DRIVER_H
