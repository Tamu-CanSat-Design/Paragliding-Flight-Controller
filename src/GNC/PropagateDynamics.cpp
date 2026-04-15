#include "GNC/PropagateDynamics.h"
#include "Common.h"
#include "GNC/MatrixUtils.h"

#include <arduino.h>
#include <BasicLinearAlgebra.h>

namespace PropagateDynamics {

void PropagateLinear(Common::StateVector& NextState, const Common::StateVector CurrentState, const Common::IMUData IMUData) {

}

void PropagateQuaternion(Common::StateVector& NextState, const Common::StateVector CurrentState, float dt) {
    BLA::Matrix<13> StateMatrix;
    MatrixUtils::StateToMatrix(CurrentState, StateMatrix);
    
    BLA::Matrix<4> Quaternion;
    BLA::Matrix<3> AngVel;
    float OmegaSum = 0;
    for (int i =0; i<4; i++){
        Quaternion(i) = StateMatrix(i+6);
    }
    for (int i =0; i < 3; i++){
        AngVel(i) = StateMatrix(10+i);
        OmegaSum += AngVel(i);
    }
    float OmegaNorm = sqrt(OmegaSum);

    BLA::Matrix<4,4> I_4 = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

    float CosVal = cos(0.5 * OmegaNorm * dt);
    float SinVal = sin(0.5 * OmegaNorm * dt);

    BLA::Matrix<4,4> Omega4 = MatrixUtils::Omega(Quaternion);
    BLA::Matrix<4> QuatNext = (CosVal * I_4 + 1/OmegaNorm * SinVal * Omega4) * Quaternion;
    MatrixUtils::NormalizeQuat(Quaternion);    
}

}
