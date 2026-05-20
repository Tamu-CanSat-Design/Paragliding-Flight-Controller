#include "TaskHandler.h"
#include "MissionManager.h"





void Tasks::MissionManagerTask(void* pvParameters){
    (void)pvParameters; // Avoid unused parameter warning
    //unlock mutex to read state
    

    bool simEnabled = false;
    Common::FlightState currentState = Common::FlightState::STANDBY;
    MissionManager::AscentStartedFlag = false;
    MissionManager::DescentStartedFlag = false;
    
    Common::StateVector LocalStateVector = {};
    Common::SensorSnapshot LocalSensorSnapshot = {};
    
    while(true){
        
        
        while(xQueueReceive(SharedData::gCommandQueue, &MissionManager::Command, 0) == pdTRUE) {
            if(xSemaphoreTake(SharedData::gStateVectorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                LocalStateVector = SharedData::gStateVector;
                if(MissionManager::MaxAltitude < LocalStateVector.z) {
                    MissionManager::MaxAltitude = LocalStateVector.z;
                }
                xSemaphoreGive(SharedData::gStateVectorMutex);
            }
            if(xSemaphoreTake(SharedData::gSnapshotMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                LocalSensorSnapshot = SharedData::gSnapshot;
                xSemaphoreGive(SharedData::gSnapshotMutex);
            }
            if(MissionManager::Command[0] == 'C' && MissionManager::Command[1] == 'X' && MissionManager::Command[2] == ',' && MissionManager::Command[3] == 'O' && MissionManager::Command[4] == 'N' && MissionManager::Command[5] == '\0')
            {
                if(!MissionManager::localCXFlag) {
                // notify telemetry to send CX packet immediately
                    if(xSemaphoreTake(SharedData::gCXFlagMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        SharedData::gCXFlag = true;
                        xSemaphoreGive(SharedData::gCXFlagMutex);
                    } 
                    MissionManager::localCXFlag = true;
                } 
            }
            if(MissionManager::Command[0] == 'C' && MissionManager::Command[1] == 'X' && MissionManager::Command[2] == ',' && MissionManager::Command[3] == 'O' && MissionManager::Command[4] == 'F' && MissionManager::Command[5] == 'F' && MissionManager::Command[6] == '\0')
            {
                if(MissionManager::localCXFlag) {
                    if(xSemaphoreTake(SharedData::gCXFlagMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        SharedData::gCXFlag = false;
                        xSemaphoreGive(SharedData::gCXFlagMutex);
                    } 
                    MissionManager::localCXFlag = false;
                } 
            }
            else if(MissionManager::Command[0] == 'S' && MissionManager::Command[1] == 'T' && MissionManager::Command[2] == ',') 
            {
                // Take rest of the command after ST as a string then send it over a queue to RTC thread to set the RTC time. If the command has GPS then tell the RTC to set time from GPS
               
                char timeArg[Common::TIME_STRING_LENGTH] = {0};
                //get rid of the "ST," part of the command
                memcpy(timeArg, MissionManager::Command + 3, sizeof(timeArg) - 1);
                timeArg[sizeof(timeArg) - 1] = '\0';

                


                xQueueSend(SharedData::gSetTimeQueue, &timeArg, 0);
            }
            else if(MissionManager::Command[0] == 'S' && MissionManager::Command[1] == 'I' && MissionManager::Command[2] == 'M' && MissionManager::Command[3] == '\0') {

                if(!simEnabled){
                    simEnabled = true;
                    
                    xTaskNotifyGive(Tasks::NavigationTaskHandle);
                    xTaskNotifyGive(Tasks::ControlTaskHandle);
                }   
                else{
                    vTaskResume(Tasks::NavigationTaskHandle);
                    vTaskResume(Tasks::ControlTaskHandle);
                    simEnabled = false;
                }
            }
            else if(MissionManager::Command[0] == 'S' && MissionManager::Command[1] == 'I' && MissionManager::Command[2] == 'M' && MissionManager::Command[3] == 'P' && MissionManager::Command[4] == '\0') {
                //Altitude calculated from pressure
            }
            else if(MissionManager::Command[0] == 'C' && MissionManager::Command[1] == 'A' && MissionManager::Command[2] == 'L' && MissionManager::Command[3] == '\0') {
                MissionManager::CAL();
            }
            else if(MissionManager::Command[0] == 'M' && MissionManager::Command[1] == 'E' && MissionManager::Command[2] == 'C' && MissionManager::Command[3] == ',' 
                && MissionManager::Command[4] == 'E' && MissionManager::Command[5] == 'G' && MissionManager::Command[6] == 'R' && MissionManager::Command[7] == 'E' 
                && MissionManager::Command[8] == 'L' && MissionManager::Command[9] == '\0') {
                //Trigger servo for egg release mechanism
                Hardware::EggInstrumentServo_.SetAngle(15);
            }
            else if(MissionManager::Command[0] == 'M' && MissionManager::Command[1] == 'E' && MissionManager::Command[2] == 'C' && MissionManager::Command[3] == ',' 
                && MissionManager::Command[4] == 'L' && MissionManager::Command[5] == 'D' && MissionManager::Command[6] == 'A' && MissionManager::Command[7] == 'R' 
                && MissionManager::Command[8] == '\0') {
                //Trigger servo for lidar/camera release mechanism
                Hardware::EggInstrumentServo_.SetAngle(20);
            }
            else if(MissionManager::Command[0] == 'M' && MissionManager::Command[1] == 'E' && MissionManager::Command[2] == 'C' && MissionManager::Command[3] == ',' 
                && MissionManager::Command[4] == 'A' && MissionManager::Command[5] == 'U' && MissionManager::Command[6] == 'D' && MissionManager::Command[7] == '\0') {
                //Trigger audio playback for recovery
                if(!MissionManager::AUDFlag) {
                    Hardware::audioBeacon_.BuzzerOn();
                    MissionManager::AUDFlag = true;
                }
                else{
                    Hardware::audioBeacon_.BuzzerOff();
                    MissionManager::AUDFlag = false;
                }
            }
            else if(MissionManager::Command[0] == 'M' && MissionManager::Command[1] == 'E' && MissionManager::Command[2] == 'C'
            && MissionManager::Command[3] == ',' && MissionManager::Command[4] == 'E' && MissionManager::Command[5] == 'K' && MissionManager::Command[6] == 'F'){

            }
        }
    

        
        currentState = MissionManager::CheckStateChange(currentState, LocalStateVector, LocalSensorSnapshot);

        if (xSemaphoreTake(SharedData::gFlightStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            SharedData::gFlightState = currentState;
            xSemaphoreGive(SharedData::gFlightStateMutex);
        }


        switch(currentState){
            case Common::FlightState::STANDBY:
                if(!MissionManager::StandbyStartedFlag) {
                    MissionManager::StandbyStartedFlag = true;
                    Hardware::audioBeacon_.BuzzerOff(); // Ensure buzzer is off at startup
                    vTaskDelay(pdMS_TO_TICKS(500)); // Short delay to ensure buzzer state is registered
                }
                Hardware::audioBeacon_.BuzzerOn(); // Start beeping in standby for power indication


                break;
            case Common::FlightState::ASCENT:
                if (!MissionManager::AscentStartedFlag) {
                    MissionManager::AscentStartedFlag = true;
                }
                break;
            case Common::FlightState::DESCENT:
                if (!MissionManager::DescentStartedFlag) {
                    MissionManager::DescentStartedFlag = true;
                }
                break;
            case Common::FlightState::PAYLOAD_RELEASE:
                if (!MissionManager::PayloadReleaseStartedFlag) {
                    MissionManager::PayloadReleaseStartedFlag = true;
                }
                break;
            case Common::FlightState::RECOVERY:
                if (!MissionManager::RecoveryStartedFlag) {
                    MissionManager::RecoveryStartedFlag = true;
                }
                break;
        }


       vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed         
    }
}
