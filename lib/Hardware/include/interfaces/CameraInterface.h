#ifndef HARDWARE_CAMERA_H
#define HARDWARE_CAMERA_H

class ICamera {
public:
    virtual ~ICamera() = default;
    virtual bool Initialize() = 0;
    virtual bool StartCapture() = 0;
    virtual bool StopCapture() = 0;
};

#endif  // HARDWARE_CAMERA_H
