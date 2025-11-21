#include "GNC/GNC.h"
#define DESCENT_KP 1.0f
#define DESCENT_KI 0.1f
#define DESCENT_KD 0.05f

#define ROLL_KP 1.5f
#define ROLL_KI 0.2f
#define ROLL_KD 0.1f
class DescentController : public IController{
public:
    void Reset() override{

    }
    float Compute(float setpoint, float measurement) override;
    void set_limits(float min, float max) override;
};


class RollController : public IController{
public:
    void Reset() override{
        return;
    }
    float Compute(float setpoint, float measurement) override{
        return 0;
    }
    void set_limits(float min, float max) override{
        return;
    }
};