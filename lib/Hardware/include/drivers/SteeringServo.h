#ifndef HARDWARE_STEERING_SERVO_H
#define HARDWARE_STEERING_SERVO_H

#include "interfaces/Servo.h"

class SteeringServo : public IServo {
public:
    bool Initialize() override;
    void SetAngle(float degrees) override;
};

#endif  // HARDWARE_STEERING_SERVO_H
