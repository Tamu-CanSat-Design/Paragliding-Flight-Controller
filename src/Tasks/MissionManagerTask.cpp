#include "TaskHandler.h"
#include "MissionManager.h"

#include <cstring>

namespace {
// Ground commands arrive as "CMD,<team_id>,<payload>"; mission logic only
// cares about the payload portion when dispatching actions.
void ExtractCommandPayload(const char* command, char* payload, size_t payloadSize) {
    if (payload == nullptr || payloadSize == 0) {
        return;
    }

    payload[0] = '\0';
    if (command == nullptr) {
        return;
    }

    const char* firstComma = std::strchr(command, ',');
    if (firstComma == nullptr) {
        return;
    }
    const char* secondComma = std::strchr(firstComma + 1, ',');
    if (secondComma == nullptr) {
        return;
    }

    std::strncpy(payload, secondComma + 1, payloadSize - 1);
    payload[payloadSize - 1] = '\0';
}
}

void Tasks::MissionManagerTask(void* pvParameters){
    (void)pvParameters;

    Common::FlightState currentState = Common::FlightState::BOOTUP;
    MissionManager::InitializeRuntimeState();
    Common::StateVector LocalStateVector = {};
    Common::SensorSnapshot LocalSensorSnapshot = {};
    char commandPayload[Common::COMMAND_BUFFER_LENGTH] = {};
    
    while(true){
        HealthMonitor::Kick(HealthMonitor::TaskId::MissionManager);

        // MissionManager owns command behavior and async control completions;
        // the task is responsible for queue I/O and dispatch timing only.
        MissionManager::HandleControlResults();
        
        while(xQueueReceive(SharedData::gCommandQueue, &MissionManager::Command, 0) == pdTRUE) {
            ExtractCommandPayload(MissionManager::Command, commandPayload, sizeof(commandPayload));

            // Publish the last parsed payload for telemetry/debug visibility.
            if(xSemaphoreTake(SharedData::gCommandMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                std::strncpy(SharedData::gCommand, commandPayload, sizeof(SharedData::gCommand) - 1);
                SharedData::gCommand[sizeof(SharedData::gCommand) - 1] = '\0';
                xSemaphoreGive(SharedData::gCommandMutex);
            }

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

            MissionManager::DispatchCommand(commandPayload);
        }
    
        currentState = MissionManager::CheckStateChange(currentState, LocalStateVector, LocalSensorSnapshot);

        if(xSemaphoreTake(SharedData::gFlightStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            SharedData::gFlightState = currentState;
            xSemaphoreGive(SharedData::gFlightStateMutex);
        }

        switch(currentState){
            case Common::FlightState::BOOTUP:
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
                break;
        }

        // Yield so lower-priority sensor and telemetry tasks continue to make
        // progress while mission logic polls its queues.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
