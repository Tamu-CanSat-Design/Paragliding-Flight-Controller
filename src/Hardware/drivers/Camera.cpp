#include "Hardware/drivers/Camera.h"

#include <Arduino.h>

#include "Common.h"

bool Camera::Initialize() {
    diagnostics_ = {};
    initialized_ = false;
    capturing_ = false;
    pinMode(Common::CAMERA_PIN, OUTPUT);
    digitalWrite(Common::CAMERA_PIN, LOW);
    initialized_ = true;
    return true;
}

// Asserts trigger HIGH; both ESP32-CAM boards begin recording simultaneously.
// In production this is called once ~1-2 s after power-up and stays HIGH.
bool Camera::StartCapture() {
    if (!initialized_) { return false; }
    digitalWrite(Common::CAMERA_PIN, HIGH);
    capturing_ = true;
    ++diagnostics_.startCount;
    return true;
}

bool Camera::StopCapture() {
    if (!initialized_) { return false; }
    digitalWrite(Common::CAMERA_PIN, LOW);
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
