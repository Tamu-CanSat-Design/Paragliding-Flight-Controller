#ifndef HARDWARE_MG90S_SERVO_H
#define HARDWARE_MG90S_SERVO_H

#include "../interfaces/ServoInterface.h"
#include <Servo.h>
#include <stdint.h>

class Mg90sServo : public IServo {
public:
    explicit Mg90sServo(uint8_t pin);

    bool Initialize() override;
    void SetAngle(float degrees) override;

private:
    uint8_t pin_;
    Servo servo_;
    bool initialized_ = false;
};

#endif  // HARDWARE_MG90S_SERVO_H
