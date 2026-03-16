#include "MissionManager.h"

#include "SharedData.h"
#include "TaskHandler.h"

#include <cstring>

namespace {
bool gSimEnabled = false;
bool gBarometerCalPending = false;

// Mission manager uses the I2C1 control queue instead of touching the
// barometer directly so the task that owns the bus also owns recalibration.
bool QueueBarometerRezeroRequest() {
    if (SharedData::gI2C1ControlQueue == nullptr) {
        return false;
    }

    const Common::I2C1ControlCommand command{
        .type = Common::I2C1ControlCommandType::kRezeroBarometer,
    };
    return xQueueSend(SharedData::gI2C1ControlQueue, &command, 0) == pdTRUE;
}
}

namespace MissionManager {

    char Command[Common::COMMAND_BUFFER_LENGTH] = {};

    bool CXFlag = false;
    bool LDARFlag = false;
    bool EGRELFlag = false;
    bool AUDFlag = false;

    bool AscentStartedFlag = false;
    bool DescentStartedFlag = false;
    bool PayloadReleaseStartedFlag = false;
    bool RecoveryStartedFlag = false;

    void InitializeRuntimeState() {
        gSimEnabled = false;
        gBarometerCalPending = false;

        CXFlag = false;
        LDARFlag = false;
        EGRELFlag = false;
        AUDFlag = false;

        AscentStartedFlag = false;
        DescentStartedFlag = false;
        PayloadReleaseStartedFlag = false;
        RecoveryStartedFlag = false;

        std::memset(Command, 0, sizeof(Command));
    }

    void HandleControlResults() {
        if (SharedData::gI2C1ControlResultQueue == nullptr) {
            return;
        }

        Common::I2C1ControlResult i2c1Result{};
        while (xQueueReceive(SharedData::gI2C1ControlResultQueue, &i2c1Result, 0) == pdTRUE) {
            if (i2c1Result.type == Common::I2C1ControlCommandType::kRezeroBarometer) {
                gBarometerCalPending = false;
                Serial.printf("MissionManager: barometer CAL %s\n", i2c1Result.success ? "PASS" : "FAIL");
            }
        }
    }

    void DispatchCommand(const char* commandPayload) {
        if (commandPayload == nullptr || commandPayload[0] == '\0') {
            return;
        }

        if (std::strcmp(commandPayload, "CX") == 0) {
            CX();
        }
        else if (std::strcmp(commandPayload, "ST") == 0) {
            ST();
        }
        else if (std::strcmp(commandPayload, "SIM") == 0) {
            SIM();
        }
        else if (std::strcmp(commandPayload, "SIMP") == 0) {
            SIMP();
        }
        else if (std::strcmp(commandPayload, "CAL") == 0) {
            CAL();
        }
        else if (std::strcmp(commandPayload, "MEC,EGREL") == 0) {
            EGREL();
        }
        else if (std::strcmp(commandPayload, "MEC,LDAR") == 0) {
            LDAR();
        }
        else if (std::strcmp(commandPayload, "MEC,AUD") == 0) {
            AUD();
        }
        else {
            Serial.printf("MissionManager: unknown command '%s'\n", commandPayload);
        }
    }

    void CX() {
        if (Tasks::WriteTelemetryTaskHandle == nullptr) {
            Serial.println("MissionManager: CX failed, telemetry task handle is null");
            return;
        }

        xTaskNotifyGive(Tasks::WriteTelemetryTaskHandle);
        CXFlag = true;
    }

    void ST() {
        // TODO: Implement state-transition command behavior.
    }

    void SIM() {
        if (Tasks::NavigationTaskHandle == nullptr || Tasks::ControlTaskHandle == nullptr) {
            Serial.println("MissionManager: SIM failed, task handle is null");
            return;
        }

        if (!gSimEnabled) {
            xTaskNotifyGive(Tasks::NavigationTaskHandle);
            xTaskNotifyGive(Tasks::ControlTaskHandle);
            gSimEnabled = true;
        }
        else {
            vTaskResume(Tasks::NavigationTaskHandle);
            vTaskResume(Tasks::ControlTaskHandle);
            gSimEnabled = false;
        }
    }

    void SIMP() {
        // TODO: Implement simulated-pressure injection command behavior.
    }

    void CAL() {
        if (gBarometerCalPending) {
            Serial.println("MissionManager: barometer CAL ignored, request already pending");
            return;
        }

        gBarometerCalPending = QueueBarometerRezeroRequest();
        Serial.printf("MissionManager: barometer CAL %s\n", gBarometerCalPending ? "QUEUED" : "QUEUE_FAIL");
    }

    void EGREL() {
        Hardware::EggInstrumentServo_.SetAngle(15);
        EGRELFlag = true;
    }

    void LDAR() {
        Hardware::EggInstrumentServo_.SetAngle(20);
        LDARFlag = !LDARFlag;
    }

    void AUD() {
        if (!AUDFlag) {
            Hardware::audioBeacon_.BuzzerOn();
            AUDFlag = true;
        }
        else {
            Hardware::audioBeacon_.BuzzerOff();
            AUDFlag = false;
        }
    }

    Common::FlightState CheckStateChange(Common::FlightState currentState, Common::StateVector ekfState, Common::SensorSnapshot sensorSnapshot) {
        (void)ekfState;
        (void)sensorSnapshot;
        // Ascent starts when acceleration/velocity thresholds are met.

        if (!gSimEnabled) {
            switch (currentState) {
                default:
                    break;
            }
        }

        // Until the mission-state transition logic is filled in, preserve the
        // existing state so MissionManagerTask has deterministic behavior.
        return currentState;
    }
}
