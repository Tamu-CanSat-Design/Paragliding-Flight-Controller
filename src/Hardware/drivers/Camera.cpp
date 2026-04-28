#include "Hardware/drivers/Camera.h"

#include <Arduino.h>

#include "Common.h"

namespace {
void TriggerCameraToggle() {
    // Shared trigger assumes ESP32-CAM firmware matches the Adafruit video-toggle pulse contract.
    digitalWrite(Common::CAMERA_TRIGGER_PIN, LOW);
    delay(Common::CAMERA_TRIGGER_PULSE_MS);
    digitalWrite(Common::CAMERA_TRIGGER_PIN, HIGH);
}
}  // namespace

bool Camera::Initialize() {
    diagnostics_ = {};
    initialized_ = false;
    capturing_ = false;
    pinMode(Common::CAMERA_TRIGGER_PIN, OUTPUT);
    digitalWrite(Common::CAMERA_TRIGGER_PIN, HIGH);
    initialized_ = true;
    return true;
}

bool Camera::StartCapture() {
    if (!initialized_ || capturing_) { return false; }
    TriggerCameraToggle();
    capturing_ = true;
    ++diagnostics_.startCount;
    return true;
}

bool Camera::StopCapture() {
    if (!initialized_ || !capturing_) { return false; }
    TriggerCameraToggle();
    capturing_ = false;
    ++diagnostics_.stopCount;
    return true;
}

Camera::Diagnostics Camera::GetDiagnostics() const {
    return diagnostics_;
}

void Camera::ClearDiagnostics() {
    diagnostics_ = {};
}
