#ifndef HARDWARE_SG90_SERVO_H
#define HARDWARE_SG90_SERVO_H

#include "../interfaces/ServoInterface.h"
#include <Servo.h>
#include <stdint.h>

class Sg90Servo : public IServo {
public:
    explicit Sg90Servo(uint8_t pin);

    bool Initialize() override;
    void SetAngle(float degrees) override;

private:
    uint8_t pin_;
    Servo servo_;
    bool initialized_ = false;
};

#endif  // HARDWARE_SG90_SERVO_H
