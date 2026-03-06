#include "TaskHandler.h"
#include "MissionManager.h"





void Tasks::MissionManagerTask(void* pvParameters){
    
    //unlock mutex to read state
    

    bool simEnabled = false;
    Common::FlightState prevState;
    Common::FlightState currentState;
    while(true){
        
        
        while(xQueueReceive(SharedData::gCommandQueue, &MissionManager::Command, 0) == pdTRUE) {
            if(MissionManager::Command[0] == 'CX') {
                if(!MissionManager::CXFlag) {
                    xTaskNotify(Tasks::WriteTelemetryTaskHandle, 0, eNoAction); // notify telemetry to send CX packet immediately
                    MissionManager::CXFlag = true;
                }
                else{
                    xTaskNotify(Tasks::WriteTelemetryTaskHandle, 0, eNoAction); // notify telemetry to suspend until next CX command
                }
                MissionManager::CX();
                
            }
            else if(MissionManager::Command[0] == 'ST') {
                MissionManager::ST();
            }
            else if(MissionManager::Command[0] == 'SIM') {
                MissionManager::SIM();
            }
            else if(MissionManager::Command[0] == 'SIMP') {
                MissionManager::SIMP();
            }
            else if(MissionManager::Command[0] == 'CAL') {
                MissionManager::CAL();
            }
            else if(MissionManager::Command == "MEC,EGREL") {
                MissionManager::EGREL();
            }
            else if(MissionManager::Command == "MEC,LDAR") {
                MissionManager::LDAR();
            }
            else if(MissionManager::Command == "MEC,AUD") {
                MissionManager::AUD();
            }
        }
    

        
        MissionManager::HandleStateTasks(prevState, currentState);
        MissionManager::CheckStateChange(currentState, SharedData::gStateVector, SharedData::gSnapshot);



        switch(SharedData::gFlightState){
            case Common::FlightState::BOOTUP:
                States::Bootup();
                
                break;
            case Common::FlightState::ASCENT:
                States::Ascent();
                if(!MissionManager::AscentStartedFlag) {
                    xTaskNotify(Tasks::NavigationTaskHandle, 0, eNoAction); // notify nav to reset EKF and start ascent logic
                    MissionManager::AscentStartedFlag = true;
                }
                
                break;
            case Common::FlightState::DESCENT:
                States::Descent();
                if(!MissionManager::DescentStartedFlag) {
                    MissionManager::DescentStartedFlag = true;
                }
                break;
            case Common::FlightState::PAYLOAD_RELEASE:
                States::PayloadRelease();
                break;
            case Common::FlightState::RECOVERY:
                States::Recovery();
                break;
            default:
                //Error: Unknown State
                break;

        }

    

                
    }
}
