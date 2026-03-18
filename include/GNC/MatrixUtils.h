#ifndef __MATRIX_UTILS_H__
#define __MATRIX_UTILS_H__

#include "Common.h"
#include <BasicLinearAlgebra.h>

namespace MatrixUtils{
    void StateToMatrix(Common::StateVector StateStruct, BLA::Matrix<13>& StateMatrix);
    void MatrixToState(Common::StateVector& StateStruct, BLA::Matrix<13> StateMatrix);
    void Vector3ToMatrix(Common::Vector3 Vector3, BLA::Matrix<3>& VectorMatrix);
    void MatrixToVector3(Common::Vector3& Vector3, BLA::Matrix<3> VectorMatrix);
};

#endif // __MATRIX_UTILS_H__