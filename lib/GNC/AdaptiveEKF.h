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

    void Predict() {

        return;
    }
    void Measure() {
        return;
    }


    void SetProcessNoiseCovariance(Common::FlightState flightState) {

        if(flightState == Common::FlightState::ASCENT) {
            processNoiseCovariance.Fill(0.1); // 


        } else if (flightState == Common::FlightState::DESCENT) {
            processNoiseCovariance.Fill(0.5); // Example value for descent


        } else if(flightState == Common::FlightState::PAYLOAD_RELEASE) {
            processNoiseCovariance.Fill(0.2); // Default value for other states
        }

    }


    void SetMeasurementNoiseCovariance(Common::FlightState flightState) {
        if(flightState == Common::FlightState::ASCENT) {
            measurementNoiseCovariance.Fill(0.1); // Example value for ascent

        } else if (flightState == Common::FlightState::DESCENT) {
            measurementNoiseCovariance.Fill(0.5); // Example value for descent

        } else if(flightState == Common::FlightState::PAYLOAD_RELEASE) {
            measurementNoiseCovariance.Fill(0.2); // Default value for other states
        }
    }


private:
    BLA::Matrix<16, 1> stateVector; // [x, y, z, vx, vy, vz, q0, q1, q2, q3, q4, accelBias, gyroBias]
    BLA::Matrix<16, 16> stateCovariance;
    BLA::Matrix<16, 16> processNoiseCovariance;
    BLA::Matrix<6, 16> measurementMatrix; // Maps state vector to measurement space
    BLA::Matrix<6, 6> measurementNoiseCovariance;
    
};