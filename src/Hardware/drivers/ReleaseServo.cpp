#include "Hardware/interfaces/ServoInterface.h"



class ReleaseServo : public IServo {
public:
    bool Initialize();
    void SetAngle(float degrees);
};




bool ReleaseServo::Initialize() {
    return true;
}

void ReleaseServo::SetAngle(float degrees) {
    (void)degrees;
}
