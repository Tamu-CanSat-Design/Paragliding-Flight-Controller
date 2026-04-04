#include "GNC/AdaptiveEKF.h"


void AdaptiveEKF::Predict(const Common::IMUData& imuData){

    switch(currentFlightState){
        case Common::FlightState::ASCENT:
            // Strapdown IMU EOMs

            
            
            // r_dot = v
            // v = g + q
            // 


            break;
        case Common::FlightState::DESCENT:
            // Implement descent-specific prediction logic using imuData
            break;
        case Common::FlightState::PAYLOAD_RELEASE:
            // Implement payload release-specific prediction logic using imuData
            break;
    }


}
template<int MEAS>
void AdaptiveEKF::UpdateEKFMeasurement(const BLA::Matrix<MEAS,15>& H, const BLA::Matrix<MEAS,1>& y, const BLA::Matrix<MEAS,MEAS>& R) {
    

    // EKF update step
    // Cross Covariance = stateCovariance * H^T
    BLA::Matrix<15, MEAS> crossCovariance = stateCovariance * BLA::MatrixTranspose(H);
    // Innovation Covariance = H * stateCovariance * H^T + measurementNoiseCovariance
    BLA::Matrix<MEAS, MEAS> innovationCovariance = H * stateCovariance * BLA::MatrixTranspose(H) + R;
    // Kalman Gain = Cross Covariance * Innovation Covariance^-1
    BLA::Matrix<15, MEAS> KalmanGain = crossCovariance * BLA::Inverse(innovationCovariance);

    BLA::Matrix<15,1> stateCorrection = KalmanGain * y;
    //Inject multiplicative correction for quaternion states to ensure unit norm constraint is maintained after update
    InjectMultiplicativeQuaternionCorrection(stateCorrection);

    // stateCovariance = (I_n - Kalman Gain * H) * stateCovariance * (I_n - Kalman Gain * H)^T + Kalman Gain * R * Kalman Gain^T
    BLA::Matrix<15,15> I = BLA::Eye<15,15>();
    BLA::Matrix<15,15> I_KH = I - KalmanGain * H;
    stateCovariance = (I - KalmanGain * H) * stateCovariance * BLA::MatrixTranspose(I_KH) + KalmanGain * R * BLA::MatrixTranspose(KalmanGain);
   
}

void AdaptiveEKF::InjectMultiplicativeQuaternionCorrection(const BLA::Matrix<15,1>& dx) {

    //Position and Velocity are additive corrections
    stateVector(0) += dx(0);
    stateVector(1) += dx(1);
    stateVector(2) += dx(2);
    stateVector(3) += dx(3);
    stateVector(4) += dx(4);
    stateVector(5) += dx(5);

    //Quaternion states are updated using multiplicative correction to maintain unit norm constraint
    float dq0 = 1.0f;
    float dq1 = 0.5 * dx(6);
    float dq2 = 0.5 * dx(7);
    float dq3 = 0.5 * dx(8);

    

    // Quaternion multiplication to apply correction
    BLA::Matrix<4,1> qCorrection = GNC::QuaternionMultiplication(stateVector.Submatrix<4,1>(6,0), {dq0, dq1, dq2, dq3});
    
    float norm = BLA::Norm(qCorrection);
    if(norm > 1e-6){
        stateVector(6) = qCorrection(0) / norm; stateVector(7) = qCorrection(1) / norm; 
        stateVector(8) = qCorrection(2) / norm; stateVector(9) = qCorrection(3) / norm; 
    }


    // Bias states are additive corrections
    for (int i = 9; i < 15; ++i) stateVector(i+1) += dx(i);
    
}







void AdaptiveEKF::SetProcessNoiseCovariance() {

        if(currentFlightState == Common::FlightState::ASCENT) {
            processNoiseCovariance.Fill(0.1); // 


        } else if (currentFlightState == Common::FlightState::DESCENT) {
            processNoiseCovariance.Fill(0.5); // Example value for descent


        } else if(currentFlightState == Common::FlightState::PAYLOAD_RELEASE) {
            processNoiseCovariance.Fill(0.2); // Default value for other states
        }
}

void AdaptiveEKF::SetMeasurementNoiseCovariance(){

    switch(currentFlightState){ // Fix this
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
            // On ground, GPS and magnetometer measurements may be more reliable, so we can reduce their measurement noise covariance
            break;

    }
}




void AdaptiveEKF::UpdateMagnetometer(const Common::MagData& magData){
    (void)magData; // Avoid unused parameter warning   


    // Tilt compensation for magnetometer readings using IMU data (accelerometer and gyroscope)

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

    UpdateEKFMeasurement(magMeasurementJacobian, MagBody - GNC::RotationInertialToBody(stateVector(6,9), GNC::magReference), magnetometerNoiseCovariance);

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

    float Denominator = 1 - 2 * (stateVector(7)*stateVector(7) + stateVector(8)*stateVector(8));

    float PredictedLidar = - stateVector(2) / Denominator;

    lidarMeasurementJacobian(0, 2) = -1 / Denominator;

    UpdateEKFMeasurement(lidarMeasurementJacobian, {lidarData.distanceMeters - PredictedLidar}, lidarNoiseCovariance);
}


void AdaptiveEKF::SetFlightState(Common::FlightState flightState) {
        // Adjust process and measurement noise covariances based on flight state
        currentFlightState = flightState;
        SetProcessNoiseCovariance();
        SetMeasurementNoiseCovariance();
}



