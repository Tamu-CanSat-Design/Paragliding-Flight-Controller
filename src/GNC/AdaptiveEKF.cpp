#include "GNC/AdaptiveEKF.h"

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
            measurementNoiseCovariance.Fill(0.1); // Example value for ascent
            break;
        case Common::FlightState::DESCENT:
            measurementNoiseCovariance.Fill(0.5); // Example value for descent
            break;
        case Common::FlightState::PAYLOAD_RELEASE:
            measurementNoiseCovariance.Fill(0.2); // Example value for payload release
            break;
    }
}


void AdaptiveEKF::Predict(const Common::IMUData& imuData){

    switch(currentFlightState){
        case Common::FlightState::ASCENT:
            // Implement ascent-specific prediction logic using imuData
            
            break;
        case Common::FlightState::DESCENT:
            // Implement descent-specific prediction logic using imuData
            break;
        case Common::FlightState::PAYLOAD_RELEASE:
            // Implement payload release-specific prediction logic using imuData
            break;
    }


}


void AdaptiveEKF::UpdateMagnetometer(const Common::MagData& magData){
    (void)magData; // Avoid unused parameter warning   
}
void AdaptiveEKF::UpdateBarometer(const Common::BarometerData& baroData){
    (void)baroData; // Avoid unused parameter warning
}
void AdaptiveEKF::UpdateGPS(const Common::GPSData& gpsData){
    (void)gpsData; // Avoid unused parameter warning
}
void AdaptiveEKF::UpdateLidar(const Common::LidarData& lidarData){
    (void)lidarData; // Avoid unused parameter warning
}


void AdaptiveEKF::SetFlightState(Common::FlightState flightState) {
        // Adjust process and measurement noise covariances based on flight state
        currentFlightState = flightState;
        SetProcessNoiseCovariance();
        SetMeasurementNoiseCovariance();
}