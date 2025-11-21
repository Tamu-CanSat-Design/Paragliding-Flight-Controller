#include "GNC/GNC.h"
#include "BasicLinearAlgebra.h"
class ParagliderEKF : public IKalmanFilter{
public:
    void Predict() override{
        return;
    }
    void Measure() override{
        return;
    }


};