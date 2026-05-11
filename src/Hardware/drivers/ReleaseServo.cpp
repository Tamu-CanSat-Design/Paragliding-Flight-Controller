#include "Hardware/drivers/ReleaseServo.h"
#include <Arduino.h>

bool ReleaseServo::Initialize() {
    servo_.attach(pin_, kMinPulseUs, kMaxPulseUs);
    Lock();
    if (!servo_.attached()) {
        status_ = ServoStatus::Detached;
        return false;
    }
    status_ = ServoStatus::Ok;
    return true;
}

void ReleaseServo::SetAngle(float degrees) {
    bool clamped = (degrees < 0.0f || degrees > 180.0f);
    if (clamped) clampCount_++;

    if (degrees < 0.0f)   degrees = 0.0f;
    if (degrees > 180.0f) degrees = 180.0f;

    currentDeg_ = degrees;
    servo_.write(static_cast<int>(degrees));
}

void ReleaseServo::SlewTo(float targetDeg, float degsPerSec) {
    if (targetDeg < 0.0f)   targetDeg = 0.0f;
    if (targetDeg > 180.0f) targetDeg = 180.0f;

    if (degsPerSec < kMinDegsPerSec) degsPerSec = kMinDegsPerSec;
    if (degsPerSec > kMaxDegsPerSec) degsPerSec = kMaxDegsPerSec;

    const float stepDeg = degsPerSec * (kStepIntervalMs / 1000.0f);
    isSlewing_ = true;

    while (true) {
        float remaining = targetDeg - currentDeg_;
        if (remaining > stepDeg) {
            SetAngle(currentDeg_ + stepDeg);
        } else if (remaining < -stepDeg) {
            SetAngle(currentDeg_ - stepDeg);
        } else {
            SetAngle(targetDeg);
            break;
        }
        slewStepCount_++;
        delay(static_cast<unsigned long>(kStepIntervalMs));
    }

    isSlewing_ = false;
    slewCount_++;
}

void ReleaseServo::Lock() {
    SetAngle(kLockedDeg);
}

void ReleaseServo::Release() {
    SetAngle(kReleasedDeg);
}

void ReleaseServo::LockSlewed(float degsPerSec) {
    SlewTo(kLockedDeg, degsPerSec);
}

void ReleaseServo::ReleaseSlewed(float degsPerSec) {
    SlewTo(kReleasedDeg, degsPerSec);
}

ServoDiagnostics ReleaseServo::GetDiagnostics() const {
    return {
        status_,
        currentDeg_,
        isSlewing_,
        clampCount_,
        slewCount_,
        slewStepCount_,
    };
}

void ReleaseServo::ResetDiagnostics() {
    clampCount_    = 0;
    slewCount_     = 0;
    slewStepCount_ = 0;
}
