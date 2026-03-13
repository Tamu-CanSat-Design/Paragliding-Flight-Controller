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
        //Ascent Starts when acceleration is above 
        // if Inertial_Velocity > 1 m/s


        if(!simEnabled){
            // Logic for checking state changes
            switch(currentState){

            }
        }
        else{

        }
    }
    

};