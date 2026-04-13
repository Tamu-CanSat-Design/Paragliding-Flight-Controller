#include "TaskHandler.h"
#include "MissionManager.h"

#include <cstring>

namespace {
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
        std::strncpy(payload, command, payloadSize - 1);
        payload[payloadSize - 1] = '\0';
        return;
    }

    const char* secondComma = std::strchr(firstComma + 1, ',');
    if (secondComma == nullptr) {
        std::strncpy(payload, command, payloadSize - 1);
        payload[payloadSize - 1] = '\0';
        return;
    }

    std::strncpy(payload, secondComma + 1, payloadSize - 1);
    payload[payloadSize - 1] = '\0';
}
}  // namespace

void Tasks::MissionManagerTask(void* pvParameters) {
    (void)pvParameters;

    Common::FlightState currentState = Common::FlightState::BOOTUP;
    Common::StateVector localStateVector = {};
    Common::SensorSnapshot localSensorSnapshot = {};
    char commandBuffer[Common::COMMAND_BUFFER_LENGTH] = {};
    char commandPayload[Common::COMMAND_BUFFER_LENGTH] = {};

    MissionManager::InitializeRuntimeState();

    while (true) {
        while (xQueueReceive(SharedData::gCommandQueue, &commandBuffer, 0) == pdTRUE) {
            ExtractCommandPayload(commandBuffer, commandPayload, sizeof(commandPayload));

            if (xSemaphoreTake(SharedData::gCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                std::strncpy(SharedData::gCommand, commandPayload, sizeof(SharedData::gCommand) - 1);
                SharedData::gCommand[sizeof(SharedData::gCommand) - 1] = '\0';
                xSemaphoreGive(SharedData::gCommandMutex);
            }

            MissionManager::DispatchCommand(commandPayload);
        }

        if (xSemaphoreTake(SharedData::gStateVectorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            localStateVector = SharedData::gStateVector;
            xSemaphoreGive(SharedData::gStateVectorMutex);
        }

        if (xSemaphoreTake(SharedData::gSnapshotMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            localSensorSnapshot = SharedData::gSnapshot;
            xSemaphoreGive(SharedData::gSnapshotMutex);
        }

        currentState = MissionManager::CheckStateChange(currentState, localStateVector, localSensorSnapshot);

        if (xSemaphoreTake(SharedData::gFlightStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            SharedData::gFlightState = currentState;
            xSemaphoreGive(SharedData::gFlightStateMutex);
        }

        switch (currentState) {
            case Common::FlightState::BOOTUP:
            case Common::FlightState::STANDBY:
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

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
