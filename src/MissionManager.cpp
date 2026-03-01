#include "MissionManager.h"
#include "TaskManager.h"
#include "Common.h"


void HandleStateTasks(Common::FlightState prevState, Common::FlightState& currentState) {
    
}



void Tasks::MissionManagerTask(void* pvParameters){
    

    switch(Common::EE_STATE){
        case Common::ASCENT:
            States::Ascent();

            break;
        case Common::DESCENT:
            States::Descent();
            break;
        case Common::PAYLOAD_RELEASE:
            States::PayloadRelease();
            break;
        case Common::RECOVERY:
            States::Recovery();
            break;
        default:
            //Error: Unknown State
            break;

    }


}