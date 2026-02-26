#ifndef HARDWARE_RELEASE_SERVO_H
#define HARDWARE_RELEASE_SERVO_H

#include "interfaces/ServoInterface.h"

class ReleaseServo : public IServo {
public:
    bool Initialize() override;
    void SetAngle(float degrees) override;
};

#endif  // HARDWARE_RELEASE_SERVO_H
