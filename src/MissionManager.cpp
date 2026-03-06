#include "MissionManager.h"
#include "SharedData.h"



namespace MissionManager {

    bool simEnabled = false;
    bool CXFlag = false;
    bool LDARFlag = false;
    bool EGRELFlag = false;
    bool AUDFlag = false; 

    void CX() {
        
        if(!CXFlag) {
            // Perform CX actions (e.g., send telemetry, activate systems)
            CXFlag = true;

        }
        else{
            CXFlag = false;
        }
    }

    void ST() {
        // Implement ST command logic

    }

    void SIM() {
        // Implement SIM command logic
    }

    void SIMP() {
        // Implement SIMP command logic
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

    void AUD() {
        if(!AUDFlag) {
            AUDFlag = true;
        }
        else{
            AUDFlag = false;
        }
    }

  





    void HandleStateTasks(Common::FlightState prevState, Common::FlightState& currentState) {

    }
    Common::FlightState CheckStateChange(Common::FlightState currentState, Common::StateVector ekfState, Common::SensorSnapshot sensorSnapshot){

    }
    

};