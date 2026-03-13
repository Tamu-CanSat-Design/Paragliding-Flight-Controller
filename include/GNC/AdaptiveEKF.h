#ifndef __ADAPTIVE_EKF_H__
#define __ADAPTIVE_EKF_H__

#include "BasicLinearAlgebra.h"
#include "Common.h"
class AdaptiveEKF {
public:

    AdaptiveEKF() {
        stateVector.Fill(0.0);
        processNoiseCovariance.Fill(0.1); // Initial process noise
        measurementMatrix.Fill(0.0); // Define based on sensor model
        measurementNoiseCovariance.Fill(0.1); // Initial measurement noise
    }

    void Initialize();
    void Reset();
    void Predict();
    void UpdateIMU(const Common::IMUData& imuData);
    void UpdateMagnetometer(const Common::MagData& magData);
    void UpdateBarometer(const Common::BarometerData& baroData);
    void UpdateGPS(const Common::GPSData& gpsData);
    void UpdateLidar(const Common::LidarData& lidarData);


    void SetFlightState(Common::FlightState flightState);

    void SetProcessNoiseCovariance(Common::FlightState flightState);


    void SetMeasurementNoiseCovariance(Common::FlightState flightState);


private:
    BLA::Matrix<13, 1> stateVector; // [x, y, z, vx, vy, vz, q0, q1, q2, q3, q4, accelBias, gyroBias]
    BLA::Matrix<13, 13> processNoiseCovariance;
    BLA::Matrix<6, 13> measurementMatrix; // [Accelerometer, Gyroscope, Magnetometer, GPS, Barometer, LiDAR]
    BLA::Matrix<6, 6> measurementNoiseCovariance;
    Common::FlightState currentFlightState;
};

#endif // __ADAPTIVE_EKF_H__