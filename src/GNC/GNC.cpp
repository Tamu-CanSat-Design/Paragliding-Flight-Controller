#include "GNC/GNC.h"



namespace GNC{
    Common::Window<10> GlideRatio;
    BLA::Matrix<3,1> GPSREFERENCE = {0.0, 0.0, 0.0}; // Placeholder value for GPS reference point (could be set to launch site coordinates)
    float barometerRef = 101325.0; // Placeholder value for barometer reference pressure (could be set to sea level pressure or launch site pressure)

    BLA::Matrix<3,1> WindVelocity = {0.0,0.0,0.0};

    BLA::Matrix<3,1> RotationBodyToInertial(const BLA::Matrix<4,1>& q, const BLA::Matrix<3,1>& v_body){

        float q0 = q(0);
        float q1 = q(1);
        float q2 = q(2);
        float q3 = q(3);

        BLA::Matrix<3,3> R;
        R(0,0) = 1 - 2*(q2*q2 + q3*q3);    R(0,1) = 2*(q1*q2 - q0*q3);      R(0,2) = 2*(q1*q3 + q0*q2);
        R(1,0) = 2*(q1*q2 + q0*q3);        R(1,1) = 1 - 2*(q1*q1 + q3*q3);  R(1,2) = 2*(q2*q3 - q0*q1);
        R(2,0) = 2*(q1*q3 - q0*q2);        R(2,1) = 2*(q2*q3 + q0*q1);      R(2,2) = 1 - 2*(q1*q1 + q2*q2);

        return R * v_body;
        
    }
    
    BLA::Matrix<3,1> RotationInertialToBody(const BLA::Matrix<4,1>& q, const BLA::Matrix<3,1>& v_inertial){

        float q0 = q(0);
        float q1 = q(1);
        float q2 = q(2);
        float q3 = q(3);

        BLA::Matrix<3,3> R;
        R(0,0) = 1 - 2*(q2*q2 + q3*q3);   R(0,1) = 2*(q1*q2 + q0*q3);         R(0,2) = 2*(q1*q3 - q0*q2);
        R(1,0) = 2*(q1*q2 - q0*q3);       R(1,1) = 1 - 2*(q1*q1 + q3*q3);     R(1,2) = 2*(q2*q3 + q0*q1);
        R(2,0) = 2*(q1*q3 + q0*q2);       R(2,1) = 2*(q2*q3 - q0*q1);         R(2,2) = 1 - 2*(q1*q1 + q2*q2);

        return R * v_inertial;
    }




    BLA::Matrix<3,1> NMEAToNED(const BLA::Matrix<3,1>& GPSVector){
        float latitude = GPSVector(0);
        float longitude = GPSVector(1);
        float altitude = GPSVector(2);

        float EccentricitySquared = 1.0f - (SEMIMINOR_AXIS*SEMIMINOR_AXIS) / (SEMIMAJOR_AXIS*SEMIMAJOR_AXIS);
        float PrimeVerticalRadiusOfCurvature = SEMIMAJOR_AXIS / sqrtf(1 - EccentricitySquared * sin(latitude) * sin(latitude));

        BLA::Matrix<3,1> ECEFVector;
        ECEFVector(0) = (PrimeVerticalRadiusOfCurvature + altitude) * cos(latitude) * cos(longitude);
        ECEFVector(1) = (PrimeVerticalRadiusOfCurvature + altitude) * cos(latitude) * sin(longitude);
        ECEFVector(2) = (PrimeVerticalRadiusOfCurvature * (1 - EccentricitySquared) + altitude) * sin(latitude);

        BLA::Matrix<3,3> R;
        R(0,0) = -sin(latitude)*cos(longitude);   R(0,1) = -sin(latitude)*sin(longitude);    R(0,2) = cos(latitude);
        R(1,0) = -sin(longitude);                 R(1,1) = cos(longitude);                   R(1,2) = 0;
        R(2,0) = -cos(latitude)*cos(longitude);   R(2,1) = -cos(latitude)*sin(longitude);    R(2,2) = -sin(latitude);

        return R * (ECEFVector - GNC::GPSREFERENCE);
    }


    BLA::Matrix<4,1> QuaternionMultiplication(const BLA::Matrix<4,1>& q, const BLA::Matrix<4,1>& r){
        float q0 = q(0);
        float q1 = q(1);
        float q2 = q(2);
        float q3 = q(3);

        float r0 = r(0);
        float r1 = r(1);
        float r2 = r(2);
        float r3 = r(3);

        BLA::Matrix<4,1> result;
        result(0) = r0*q0 - r1*q1 - r2*q2 - r3*q3;
        result(1) = r0*q1 + r1*q0 - r2*q3 + r3*q2;
        result(2) = r0*q2 + r1*q3 + r2*q0 - r3*q1;
        result(3) = r0*q3 - r1*q2 - r2*q1 + r3*q0;

        return result * 1.0f;

        
    }

    BLA::Matrix<3,3> SkewSymmetric(const BLA::Matrix<3,1>& v){
        BLA::Matrix<3,3> result;
        result(0,0) = 0;       result(0,1) = -v(2);   result(0,2) = v(1);
        result(1,0) = v(2);    result(1,1) = 0;       result(1,2) = -v(0);
        result(2,0) = -v(1);   result(2,1) = v(0);    result(2,2) = 0;
        return result * 1.0f;
    }

    float wrap_pi(float angle){
        angle = fmodf(angle + PI, 2.0f * PI);

        if (angle <= 0.0f)
        angle += 2.0f * PI;

        return angle - PI;
    };
}
