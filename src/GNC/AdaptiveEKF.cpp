#include "GNC/AdaptiveEKF.h"

AdaptiveEKF::AdaptiveEKF(){
        stateVector.Fill(0.0);
        processNoiseCovariance.Fill(0.1f); // Initial process noise
        // measurementNoiseCovariance.Fill(0.1f); // Initial measurement noise
        
        // Constant Barometer Jacobian (linear measurement)
        barometerMeasurementJacobian.Fill(0.0); 
        barometerMeasurementJacobian(0, 2) = -1.0f; // Barometer measures altitude (z position)

        // Constant GPS Jacobian (linear measurement)
        gpsMeasurementJacobian.Fill(0.0); 
        gpsMeasurementJacobian(0, 0) = 1.0f; gpsMeasurementJacobian(1, 1) = 1.0f; gpsMeasurementJacobian(2, 2) = 1.0f; // GPS measures x, y, z position

        hprProcessJacobian.Fill(0.0);
        paragliderProcessJacobian.Fill(0.0);

        
}



void AdaptiveEKF::Predict(const Common::IMUData& imuData, float dt){
    if (dt <= 0.0f) {
        return;
    }
    const float ax = imuData.acceleration.x - stateVector(10); // Subtract accel bias estimates from state vector
    const float ay = imuData.acceleration.y - stateVector(11);
    const float az = imuData.acceleration.z - stateVector(12);
    const float wx = imuData.gyroVel.x - stateVector(13); // Subtract gyro bias estimates from state vector
    const float wy = imuData.gyroVel.y - stateVector(14);
    const float wz = imuData.gyroVel.z - stateVector(15);

    const float dq0 = 1.0f;
    const float dq1 = 0.5f * wx * dt;
    const float dq2 = 0.5f * wy * dt;
    const float dq3 = 0.5f * wz * dt;

    const BLA::Matrix<4,1> qNew = GNC::QuaternionMultiplication(stateVector.Submatrix<4,1>(6,0), {dq0, dq1, dq2, dq3});

    const float norm = BLA::Norm(qNew);
    if(norm > GNC::kQuatNormThreshold){
        stateVector(6) = qNew(0) / norm; stateVector(7) = qNew(1) / norm; 
        stateVector(8) = qNew(2) / norm; stateVector(9) = qNew(3) / norm; 
    }

    // Velocity integration
    const BLA::Matrix<3,1> NavAcceleration = GNC::RotationBodyToInertial(stateVector.Submatrix<4,1>(6,0), {ax, ay, az}); // Update velocity states in state vector using current quaternion estimate for rotation and accel measurements from IMU
    stateVector(3) += NavAcceleration(0) * dt;
    stateVector(4) += NavAcceleration(1) * dt;
    stateVector(5) += (NavAcceleration(2) - GNC::STANDARD_GRAVITY) * dt;

    // Position integration
    stateVector(0) += stateVector(3) * dt;
    stateVector(1) += stateVector(4) * dt;
    stateVector(2) += stateVector(5) * dt;


    const BLA::Matrix<3,3> accelSkew = -GNC::SkewSymmetric(GNC::RotationBodyToInertial(stateVector.Submatrix<4,1>(6,0), {ax, ay, az}));
    const BLA::Matrix<3,1> bodyAccelerationBias = -GNC::RotationBodyToInertial(stateVector.Submatrix<4,1>(6,0), {1.0f, 1.0f, 1.0f});
    const BLA::Matrix<3,3> gyroSkew = -GNC::SkewSymmetric({wx, wy, wz});
    switch(currentFlightState){
        case Common::FlightState::STANDBY:
            return;
        case Common::FlightState::ASCENT:
            // Strapdown IMU EOMs

            

            // ddx/ddvx = 1                  //ddy/ddvy = 1                   //ddz/ddvz = 1
            // ddv/ddtheta = skew(R_body_to_inertial * acceleration measurement from IMU)
            // ddthetadot/ddtheta = skew(angular velocity measurement from IMU)
            

            hprProcessJacobian(0,3) = 1.0f; hprProcessJacobian(1,4) = 1.0f;  hprProcessJacobian(2,5) = 1.0f;
            hprProcessJacobian.Submatrix<3,3>(3,3) = accelSkew;

            
            hprProcessJacobian(3, 9) = bodyAccelerationBias(0);
            hprProcessJacobian(4, 10) = bodyAccelerationBias(1);
            hprProcessJacobian(5, 11) = bodyAccelerationBias(2);

            // ddthetadot/ddtheta = skew(angular velocity measurement from IMU)
            
            hprProcessJacobian.Submatrix<3,3>(6,6) = gyroSkew;

            //ddthetadot/dbg
            hprProcessJacobian(6,12) = -1.0f; hprProcessJacobian(7,13) = -1.0f; hprProcessJacobian(8,14) = -1.0f;

            // dba/dba = identity
            hprProcessJacobian(9, 9) = 1.0f; hprProcessJacobian(10, 10) = 1.0f; hprProcessJacobian(11, 11) = 1.0f;
            // dbg/dbg = identity
            hprProcessJacobian(12, 12) = 1.0f; hprProcessJacobian(13, 13) = 1.0f; hprProcessJacobian(14, 14) = 1.0f;


            break;
        case Common::FlightState::DESCENT:
            // Implement descent-specific prediction logic using imuData
            break;
        case Common::FlightState::PAYLOAD_RELEASE:
            // Implement payload release-specific prediction logic using imuData
            break;
        case Common::FlightState::RECOVERY:
            return;
    }

    const BLA::Matrix<15,15> Phi = GNC::kIdentity + hprProcessJacobian * dt; // State transition matrix for discrete-time EKF
    const BLA::Matrix<15,15> Qd = processNoiseCovariance * dt; // Discrete process noise covariance
    errorStateCovariance = Phi * errorStateCovariance * BLA::MatrixTranspose(Phi) + Qd;


}
template<int MEAS>
void AdaptiveEKF::UpdateEKFMeasurement(const BLA::Matrix<MEAS,15>& H, const BLA::Matrix<MEAS,1>& y, const BLA::Matrix<MEAS,MEAS>& R) {
    

    // EKF update step
    // Cross Covariance = errorStateCovariance * H^T
    const BLA::Matrix<15, MEAS> crossCovariance = errorStateCovariance * BLA::MatrixTranspose(H);
    // Innovation Covariance = H * stateCovariance * H^T + measurementNoiseCovariance
    const BLA::Matrix<MEAS, MEAS> innovationCovariance = H * crossCovariance + R;
    // Kalman Gain = Cross Covariance * Innovation Covariance^-1
    BLA::Matrix<15, MEAS> KalmanGain = crossCovariance * BLA::Inverse(innovationCovariance);

    //Inject multiplicative correction for quaternion states to ensure unit norm constraint is maintained after update
    InjectMultiplicativeQuaternionCorrection(KalmanGain * y);

    // stateCovariance = (I_n - Kalman Gain * H) * stateCovariance * (I_n - Kalman Gain * H)^T + Kalman Gain * R * Kalman Gain^T
    BLA::Matrix<15,15> IKH = GNC::kIdentity - KalmanGain * H;
    errorStateCovariance = IKH * errorStateCovariance * BLA::MatrixTranspose(IKH) + KalmanGain * R * BLA::MatrixTranspose(KalmanGain);
   
}

void AdaptiveEKF::InjectMultiplicativeQuaternionCorrection(const BLA::Matrix<15,1>& dx) {

    //Position and Velocity are additive corrections
    for(int i = 0; i < 6; ++i) stateVector(i) += dx(i);

    //Quaternion states are updated using multiplicative correction to maintain unit norm constraint
    const float dq0 = 1.0f;
    const float dq1 = 0.5f * dx(6);
    const float dq2 = 0.5f * dx(7);
    const float dq3 = 0.5f * dx(8);

    

    // Quaternion multiplication to apply correction
    const BLA::Matrix<4,1> qCorrection = GNC::QuaternionMultiplication(stateVector.Submatrix<4,1>(6,0), {dq0, dq1, dq2, dq3});
    
    const float norm = BLA::Norm(qCorrection);
    if(norm > GNC::kQuatNormThreshold){
        stateVector(6) = qCorrection(0) / norm; stateVector(7) = qCorrection(1) / norm; 
        stateVector(8) = qCorrection(2) / norm; stateVector(9) = qCorrection(3) / norm; 
    }


    // Bias states are additive corrections
    for (int i = 9; i < 15; ++i) stateVector(i+1) += dx(i);
    
}







void AdaptiveEKF::SetProcessNoiseCovariance() {

        if(currentFlightState == Common::FlightState::ASCENT) {
            processNoiseCovariance.Fill(0.1f); // 


        } else if (currentFlightState == Common::FlightState::DESCENT) {
            processNoiseCovariance.Fill(0.5f); // Example value for descent


        } else if(currentFlightState == Common::FlightState::PAYLOAD_RELEASE) {
            processNoiseCovariance.Fill(0.2f); // Default value for other states
        }
}

void AdaptiveEKF::SetMeasurementNoiseCovariance(){

    switch(currentFlightState){ // Fix this
        case Common::FlightState::STANDBY:
            return;
        case Common::FlightState::ASCENT:
            // measurementNoiseCovariance.Fill(0.1); // Example value for ascent
            break;
        case Common::FlightState::DESCENT:
            // measurementNoiseCovariance.Fill(0.5); // Example value for descent
            break;
        case Common::FlightState::PAYLOAD_RELEASE:
            // measurementNoiseCovariance.Fill(0.2); // Example value for payload release
            break;
        case Common::FlightState::RECOVERY:
            return;
    }
}




void AdaptiveEKF::UpdateMagnetometer(const Common::MagData& magData){
    (void)magData; // Avoid unused parameter warning   


    // Tilt compensation for magnetometer readings using IMU data (accelerometer and gyroscope)
    const BLA::Matrix<3,1> MagPrediction = GNC::RotationInertialToBody(stateVector(6,9), GNC::kMagReference);
    // Do rotation from magnetometer reference vector reading from navigation to body frame using current quaternion estimate from state vector
    BLA::Matrix<3,1> MagBody;
    MagBody(0) = magData.magX;
    MagBody(1) = magData.magY;
    MagBody(2) = magData.magZ;

    // Compute measurement Jacobian H for magnetometer measurement update (Placeholder)
    magMeasurementJacobian(0, 0) = 0; magMeasurementJacobian(0, 1) = 0; magMeasurementJacobian(0, 2) = 0; // Partial derivatives of mag measurement w.r.t position
    magMeasurementJacobian(1, 0) = 0; magMeasurementJacobian(1, 1) = 0; magMeasurementJacobian(1, 2) = 0; // Partial derivatives of mag measurement w.r.t position
    magMeasurementJacobian(2, 0) = 0; magMeasurementJacobian(2, 1) = 0; magMeasurementJacobian(2, 2) = 0; // Partial derivatives of mag measurement w.r.t position

    // Take derivative of rotation matrix with respect to quaternion elements to get measurement matrix H

    UpdateEKFMeasurement(magMeasurementJacobian, MagBody - MagPrediction, magnetometerNoiseCovariance);

}
void AdaptiveEKF::UpdateBarometer(const Common::BarometerData& baroData){
    

    UpdateEKFMeasurement(barometerMeasurementJacobian, {baroData.altitude - stateVector(2)}, barometerNoiseCovariance);

}
void AdaptiveEKF::UpdateGPS(const Common::GPSData& gpsData){

    //TODO: Account for GPS velocity coming from NMEA frame and convert to NED frame for measurement update
    // gives speed over ground and course over ground which can be converted to velocity in NED frame using current heading from state vector quaternion


    // Convert GPS lat/lon to local NED frame
    BLA::Matrix<3,1> gpsNED = GNC::NMEAToNED({gpsData.latitude, gpsData.longitude, gpsData.altitude});

    
    // Update state vector with GPS position and velocity measurements
    UpdateEKFMeasurement(gpsMeasurementJacobian, gpsNED - stateVector.Submatrix<3,1>(0,0), gpsNoiseCovariance);
}
void AdaptiveEKF::UpdateLidar(const Common::LidarData& lidarData){

    //TODO: Account for lidar position offset from center of mass in measurement model

    const float Denominator = 1.f - 2 * (stateVector(7)*stateVector(7) + stateVector(8)*stateVector(8));

    const float PredictedLidar = - stateVector(2) / Denominator;

    lidarMeasurementJacobian(0, 2) = -1.f / Denominator;

    UpdateEKFMeasurement(lidarMeasurementJacobian, {lidarData.distanceMeters - PredictedLidar}, lidarNoiseCovariance);
}


void AdaptiveEKF::SetFlightState(Common::FlightState flightState) {
        // Adjust process and measurement noise covariances based on flight state
        currentFlightState = flightState;
        SetProcessNoiseCovariance();
        SetMeasurementNoiseCovariance();
}


