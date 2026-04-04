#ifndef __GNC_H__
#define __GNC_H__

#include <BasicLinearAlgebra.h>
#include "Common.h"
namespace GNC{

    // Calibration
    inline constexpr float MAGREFX = 0.0; // Placeholder values for magnetometer calibration
    inline constexpr float MAGREFY = 0.0;
    inline constexpr float MAGREFZ = 0.0;
    inline const BLA::Matrix<3,1> magReference = {MAGREFX, MAGREFY, MAGREFZ};

    inline constexpr float SEMIMAJOR_AXIS = 6378137.0; // Placeholder values for accelerometer calibration
    inline constexpr float SEMIMINOR_AXIS = 6356752.3142; // Placeholder values for accelerometer calibration

    
    extern BLA::Matrix<3,1> GPSREFERENCE; 

    extern float barometerRef;

    BLA::Matrix<3,1> RotationBodyToInertial(const BLA::Matrix<4,1> q, const BLA::Matrix<3,1> v_body);
    
    BLA::Matrix<3,1> RotationInertialToBody(const BLA::Matrix<4,1> q, const BLA::Matrix<3,1> v_inertial);

    BLA::Matrix<3,1> NMEAToNED(BLA::Matrix<3,1> GPSVector);

    BLA::Matrix<4,1> QuaternionMultiplication(const BLA::Matrix<4,1> q1, const BLA::Matrix<4,1> q2);
}

#endif // __GNC_H__