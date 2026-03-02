#ifndef HARDWARE_SERVO_H
#define HARDWARE_SERVO_H

class IServo {
public:
    virtual ~IServo() = default;
    virtual bool Initialize() = 0;
    virtual void SetAngle(float degrees) = 0;
};

#endif  // HARDWARE_SERVO_H
