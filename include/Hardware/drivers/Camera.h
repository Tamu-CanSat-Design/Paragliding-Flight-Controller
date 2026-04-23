#ifndef HARDWARE_CAMERA_DRIVER_H
#define HARDWARE_CAMERA_DRIVER_H

#include <cstdint>

#include "../interfaces/CameraInterface.h"

// GPIO trigger driver for two ESP32-CAM boards sharing one trigger line.
// StartCapture() asserts the trigger HIGH; both cameras begin recording.
// StopCapture() deasserts LOW. In production, StartCapture() is called once
// ~1-2 s after power-up and the pin stays HIGH for the full mission duration.
class Camera : public ICamera {
public:
    struct Diagnostics {
        uint32_t startCount = 0;
        uint32_t stopCount = 0;
    };

    static constexpr const char* kSensorName = "ESP32-CAM";

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
