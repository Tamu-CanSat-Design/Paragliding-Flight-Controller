#ifndef __ADAPTIVE_EKF_H__
#define __ADAPTIVE_EKF_H__

#include "BasicLinearAlgebra.h"
#include "Common.h"
#include "GNC/GNC.h"
class AdaptiveEKF {
public:

    AdaptiveEKF() {
        stateVector.Fill(0.0);
        processNoiseCovariance.Fill(0.1); // Initial process noise
        // measurementNoiseCovariance.Fill(0.1); // Initial measurement noise
        
        // Constant Barometer Jacobian (linear measurement)
        barometerMeasurementJacobian.Fill(0.0); 
        barometerMeasurementJacobian(0, 2) = 1.0; // Barometer measures altitude (z position)

        // Constant GPS Jacobian (linear measurement)
        gpsMeasurementJacobian.Fill(0.0); 
        gpsMeasurementJacobian(0, 0) = 1.0; gpsMeasurementJacobian(1, 1) = 1.0; gpsMeasurementJacobian(2, 2) = 1.0; // GPS measures x, y, z position


    }

    void Reset();
    void Predict(const Common::IMUData& imuData);
    template<int MEAS>
    void UpdateEKFMeasurement(const BLA::Matrix<MEAS,15>& H, const BLA::Matrix<MEAS,1>& y, const BLA::Matrix<MEAS,MEAS>& R);
    void InjectMultiplicativeQuaternionCorrection(const BLA::Matrix<15,1>& dx);
    void UpdateMagnetometer(const Common::MagData& magData);
    void UpdateBarometer(const Common::BarometerData& baroData);
    void UpdateGPS(const Common::GPSData& gpsData);
    void UpdateLidar(const Common::LidarData& lidarData);


    void SetFlightState(Common::FlightState flightState);

    void SetProcessNoiseCovariance();


    void SetMeasurementNoiseCovariance();

    Common::StateVector GetStateVector() {
        Common::StateVector CommonState;
        CommonState.x = stateVector(0);
        CommonState.y = stateVector(1);
        CommonState.z = stateVector(2);
        CommonState.vx = stateVector(3);
        CommonState.vy = stateVector(4);
        CommonState.vz = stateVector(5);
        CommonState.q0 = stateVector(6);
        CommonState.q1 = stateVector(7);
        CommonState.q2 = stateVector(8);
        CommonState.q3 = stateVector(9);
        return CommonState;
    }

private:
    BLA::Matrix<16, 1> stateVector; // [x, y, z, vx, vy, vz, q0, q1, q2, q3, accelBiasx, accelBiasy, accelBiasz, gyroBiasx, gyroBiasy, gyroBiasz]
    BLA::Matrix<15, 15> stateCovariance;
    
    BLA::Matrix<15, 15> processNoiseCovariance;
    BLA::Matrix<3, 3> magnetometerNoiseCovariance;
    BLA::Matrix<3, 3> gpsNoiseCovariance;
    BLA::Matrix<1, 1> barometerNoiseCovariance;
    BLA::Matrix<1, 1> lidarNoiseCovariance;


    BLA::Matrix<3,15> magMeasurementJacobian;
    BLA::Matrix<3,15> gpsMeasurementJacobian;
    BLA::Matrix<1,15> barometerMeasurementJacobian;
    BLA::Matrix<1,15> lidarMeasurementJacobian;

    
    Common::FlightState currentFlightState;
};

#endif // __ADAPTIVE_EKF_H__