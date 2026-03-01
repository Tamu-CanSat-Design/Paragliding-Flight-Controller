#include "interfaces/CameraInterface.h"

class Camera : public ICamera {
public:
    bool Initialize();
    bool StartCapture();
    bool StopCapture();
};

bool Camera::Initialize() {
    return true;
}

bool Camera::StartCapture() {
    return true;
}

bool Camera::StopCapture() {
    return true;
}
