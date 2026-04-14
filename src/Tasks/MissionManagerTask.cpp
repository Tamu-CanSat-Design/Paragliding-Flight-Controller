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
            if(MissionManager::Command[0] == 'C' && MissionManager::Command[1] == 'X' && MissionManager::Command[2] == '\0') {
                if(!MissionManager::CXFlag) {
                    xTaskNotifyGive(Tasks::WriteTelemetryTaskHandle); // notify telemetry to send CX packet immediately
                    MissionManager::CXFlag = true;
                }
                else{
                    xTaskNotifyGive(Tasks::WriteTelemetryTaskHandle); // notify telemetry to suspend until next CX command
                }   
            }
            else if(MissionManager::Command[0] == 'S' && MissionManager::Command[1] == 'T' && MissionManager::Command[2] == '\0') {
                MissionManager::ST();
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
                //Calibrate altitude to be 0 by uploading current pressure and writing to EEPROM
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
        }
    

        
        currentState = MissionManager::CheckStateChange(currentState, LocalStateVector, LocalSensorSnapshot);

        if(xSemaphoreTake(SharedData::gFlightStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            SharedData::gFlightState = currentState;
            xSemaphoreGive(SharedData::gFlightStateMutex);
        }


        switch(currentState){
            case Common::FlightState::STANDBY:
                
                break;
            case Common::FlightState::ASCENT:
                if(!MissionManager::AscentStartedFlag) {
                    //write to eeprom
                    MissionManager::AscentStartedFlag = true;
                }
                
                break;
            case Common::FlightState::DESCENT:
                if(!MissionManager::DescentStartedFlag) {
                    //write to eeprom
                    MissionManager::DescentStartedFlag = true;
                }
                break;
            case Common::FlightState::PAYLOAD_RELEASE:
                if(!MissionManager::PayloadReleaseStartedFlag) {
                    //write to eeprom
                    MissionManager::PayloadReleaseStartedFlag = true;
                }
                // Egg drop logic
                break;
            case Common::FlightState::RECOVERY:
                if(!MissionManager::RecoveryStartedFlag) {
                    //write to eeprom
                    //start audio beacon
                    MissionManager::RecoveryStartedFlag = true;
                }

                break;
            default:
                //Error: Unknown State
                break;

        }


       vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed         
    }
}
