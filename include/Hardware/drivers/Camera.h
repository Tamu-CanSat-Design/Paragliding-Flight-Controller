#ifndef HARDWARE_CAMERA_DRIVER_H
#define HARDWARE_CAMERA_DRIVER_H

#include <cstdint>

#include "../interfaces/CameraInterface.h"

// Shared trigger driver for the ESP32-CAM and Adafruit Mini Spy Camera paths.
// Both cameras use one idle-HIGH GPIO and active-LOW pulse for start/stop.
class Camera : public ICamera {
public:
    struct Diagnostics {
        uint32_t startCount = 0;
        uint32_t stopCount = 0;
    };

    static constexpr const char* kSensorName = "ESP32-CAM + Mini Spy Camera";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    bool StartCapture() override;
    bool StopCapture() override;
    Diagnostics GetDiagnostics() const;
    void ClearDiagnostics();

private:
    Diagnostics diagnostics_{};
    bool initialized_ = false;
    bool capturing_ = false;
};

#endif  // HARDWARE_CAMERA_DRIVER_H
