#include "interfaces/ServoInterface.h"


class SteeringServo : public IServo {
public:
    bool Initialize();
    void SetAngle(float degrees);
};

bool SteeringServo::Initialize() {
    return true;
}

void SteeringServo::SetAngle(float degrees) {
    (void)degrees;
}
