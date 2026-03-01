#include "AdaptiveEKF.h"

void AdaptiveEKF::SetProcessNoiseCovariance(Common::FlightState flightState) {

        if(flightState == Common::FlightState::ASCENT) {
            processNoiseCovariance.Fill(0.1); // 


        } else if (flightState == Common::FlightState::DESCENT) {
            processNoiseCovariance.Fill(0.5); // Example value for descent


        } else if(flightState == Common::FlightState::PAYLOAD_RELEASE) {
            processNoiseCovariance.Fill(0.2); // Default value for other states
        }

    }