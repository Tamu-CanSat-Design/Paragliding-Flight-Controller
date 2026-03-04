#include "BasicLinearAlgebra.h"
#include "Common.h"
class AdaptiveEKF {
public:

    AdaptiveEKF() {
        stateVector.Fill(0.0);
        stateCovariance.Fill(1.0); // Initial uncertainty
        processNoiseCovariance.Fill(0.1); // Initial process noise
        measurementMatrix.Fill(0.0); // Define based on sensor model
        measurementNoiseCovariance.Fill(0.1); // Initial measurement noise
    }

    void Initialize() {
        // Initialize state vector and covariance based on initial conditions
        stateVector.Fill(0.0);
        stateCovariance.Fill(1.0);
    }

    void Predict();
    void UpdateIMU(const Common::IMUData& imuData);
    void UpdateMagnetometer(const Common::MagData& magData);
    void UpdateBarometer(const Common::BarometerData& baroData);
    void UpdateGPS(const Common::GPSData& gpsData);
    void UpdateLidar(const Common::LidarData& lidarData);


    void SetFlightState(Common::FlightState flightState) {
        // Adjust process and measurement noise covariances based on flight state
        currentFlightState = flightState;
        SetProcessNoiseCovariance(flightState);
        SetMeasurementNoiseCovariance(flightState);
    }

    void SetProcessNoiseCovariance(Common::FlightState flightState);


    void SetMeasurementNoiseCovariance(Common::FlightState flightState);


private:
    BLA::Matrix<16, 1> stateVector; // [x, y, z, vx, vy, vz, q0, q1, q2, q3, q4, accelBias, gyroBias]
    BLA::Matrix<16, 16> stateCovariance;
    BLA::Matrix<16, 16> processNoiseCovariance;
    BLA::Matrix<6, 16> measurementMatrix; // Maps state vector to measurement space
    BLA::Matrix<6, 6> measurementNoiseCovariance;
    Common::FlightState currentFlightState;
};