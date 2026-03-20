#include "MissionManager.h"
#include "SharedData.h"



namespace MissionManager {


    char Command[32];

    bool simEnabled = false;
    bool CXFlag = false;
    bool LDARFlag = false;
    bool EGRELFlag = false;
    bool AUDFlag = false; 
    
    bool AscentStartedFlag = false;
    bool DescentStartedFlag = false;
    bool PayloadReleaseStartedFlag = false;
    bool RecoveryStartedFlag = false;
    float MaxAltitude = 0.0;

    

    

    void ST() {
        // Implement ST command logic

    }

    void SIM() {
        // Implement SIM command logic
    }

    void SIMP() {
        // Implement SIMP command logic
        // Turn pressure into an altitude
        
    }

    void CAL() {
        // Implement CAL command logic
    }

    void EGREL() {
        EGRELFlag = true;
    }

    void LDAR() {
        if(!LDARFlag) {
            LDARFlag = true;
        }
        else{
            LDARFlag = false;
        }
    }

    

  





    
    Common::FlightState CheckStateChange(Common::FlightState currentState, Common::StateVector ekfState, Common::SensorSnapshot sensorSnapshot){
        //Ascent Starts when velocity is above 1 m/s
        // if Inertial_Velocity > 1 m/s



        if(!simEnabled){
            // Logic for checking state changes
            switch(currentState){
                case Common::FlightState::BOOTUP:
                    if(ekfState.vz > 1.0f) {
                        return Common::FlightState::ASCENT;
                    }
                    break;
                case Common::FlightState::ASCENT:
                    if(ekfState.vz < 0.0f && sensorSnapshot.barometerData.altitude > 10.0f) { // Using a sliding window, make sure altitude starts decreasing
                        return Common::FlightState::DESCENT;
                    }
                    break;
                case Common::FlightState::DESCENT:
                    if(sensorSnapshot.barometerData.altitude < 10.0f) { //store the max altitude, if 80% of max altitude is reached, trigger payload release
                        //check with barometer sliding window and with extended kalman filter altitude (ekfstate.z)
                        return Common::FlightState::PAYLOAD_RELEASE;
                    }
                    break;
                case Common::FlightState::PAYLOAD_RELEASE:
                    // Check if sensor data is static (e.g., low acceleration, low gyro rates) for a certain duration to confirm landing
                    break;
                case Common::FlightState::RECOVERY:
                    // Final state, may not transition out of this
                    break;
                default:
                    // Handle unexpected state
                    break;
            }
        }
        else{

        }
    }
    

};