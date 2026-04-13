#include "MissionManager.h"

#include "SharedData.h"
#include "TaskHandler.h"

#include <cmath>
#include <cstring>
#include <cstdlib>

namespace {
void SetSharedSimEnabled(bool enabled) {
    if (SharedData::gSimEnabledMutex == nullptr) {
        return;
    }

    if (xSemaphoreTake(SharedData::gSimEnabledMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        SharedData::gSimEnabled = enabled;
        xSemaphoreGive(SharedData::gSimEnabledMutex);
    }
}

bool TryGetCurrentBarometerPressure(float* outPressureHpa) {
    if (outPressureHpa == nullptr || SharedData::gSnapshotMutex == nullptr) {
        return false;
    }

    if (xSemaphoreTake(SharedData::gSnapshotMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }

    const float pressure = SharedData::gSnapshot.barometerData.pressure;
    xSemaphoreGive(SharedData::gSnapshotMutex);

    if (pressure <= 0.0f) {
        return false;
    }

    *outPressureHpa = pressure;
    return true;
}
}  // namespace

namespace MissionManager {

char Command[Common::COMMAND_BUFFER_LENGTH] = {};

bool simArmed = false;
bool simEnabled = false;
bool CXFlag = false;
bool LDARFlag = false;
bool EGRELFlag = false;
bool AUDFlag = false;

bool AscentStartedFlag = false;
bool DescentStartedFlag = false;
bool PayloadReleaseStartedFlag = false;
bool RecoveryStartedFlag = false;
float MaxAltitude = 0.0f;

static Common::Window<10> altWindow;
static Common::Window<10> vzWindow;
static Common::Window<10> accelWindow;
static Common::Window<10> gyroWindow;

void InitializeRuntimeState() {
    std::memset(Command, 0, sizeof(Command));

    simArmed = false;
    simEnabled = false;
    SetSharedSimEnabled(false);

    CXFlag = false;
    LDARFlag = false;
    EGRELFlag = false;
    AUDFlag = false;

    AscentStartedFlag = false;
    DescentStartedFlag = false;
    PayloadReleaseStartedFlag = false;
    RecoveryStartedFlag = false;
    MaxAltitude = 0.0f;

    altWindow.clear();
    vzWindow.clear();
    accelWindow.clear();
    gyroWindow.clear();
}

void DispatchCommand(const char* commandPayload) {
    if (commandPayload == nullptr || commandPayload[0] == '\0') {
        return;
    }

    std::strncpy(Command, commandPayload, sizeof(Command) - 1);
    Command[sizeof(Command) - 1] = '\0';

    if (std::strcmp(Command, "CX") == 0) {
        CX();
    } else if (std::strncmp(Command, "ST", 2) == 0) {
        ST();
    } else if (std::strncmp(Command, "SIMP", 4) == 0) {
        SIMP();
    } else if (std::strncmp(Command, "SIM", 3) == 0) {
        SIM();
    } else if (std::strcmp(Command, "CAL") == 0) {
        CAL();
    } else if (std::strcmp(Command, "MEC,EGREL") == 0) {
        EGREL();
    } else if (std::strcmp(Command, "MEC,LDAR") == 0) {
        LDAR();
    } else if (std::strcmp(Command, "MEC,AUD") == 0) {
        AUD();
    }
}

void CX() {
    if (Tasks::WriteTelemetryTaskHandle == nullptr) {
        return;
    }

    xTaskNotifyGive(Tasks::WriteTelemetryTaskHandle);
    CXFlag = true;
}

void ST() {
    // Mission-time source selection is still a placeholder on this branch.
}

void SIM() {
    const char* token = std::strchr(Command, ',');
    if (token == nullptr) {
        simEnabled = !simEnabled;
        simArmed = simEnabled;
        SetSharedSimEnabled(simEnabled);
        return;
    }

    ++token;
    if (std::strcmp(token, "ENABLE") == 0) {
        simArmed = true;
        simEnabled = false;
    } else if (std::strcmp(token, "ACTIVATE") == 0) {
        if (simArmed) {
            simEnabled = true;
        }
    } else if (std::strcmp(token, "DISABLE") == 0) {
        simEnabled = false;
        simArmed = false;
    }

    SetSharedSimEnabled(simEnabled);
}

void SIMP() {
    // SIMP remains a mission-layer stub in this cleanup pass.
}

void CAL() {
    float pressureHpa = 0.0f;
    if (!TryGetCurrentBarometerPressure(&pressureHpa) || SharedData::gReferencePressureMutex == nullptr) {
        return;
    }

    if (xSemaphoreTake(SharedData::gReferencePressureMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        SharedData::gReferencePressureHpa = pressureHpa;
        xSemaphoreGive(SharedData::gReferencePressureMutex);
    }

    MaxAltitude = 0.0f;
    altWindow.clear();
    vzWindow.clear();
    accelWindow.clear();
    gyroWindow.clear();
}

void EGREL() {
    Hardware::EggInstrumentServo_.SetAngle(15);
    EGRELFlag = !EGRELFlag;
}

void LDAR() {
    Hardware::EggInstrumentServo_.SetAngle(20);
    LDARFlag = !LDARFlag;
}

void AUD() {
    if (!AUDFlag) {
        Hardware::audioBeacon_.BuzzerOn();
        AUDFlag = true;
    } else {
        Hardware::audioBeacon_.BuzzerOff();
        AUDFlag = false;
    }
}

Common::FlightState CheckStateChange(Common::FlightState currentState,
                                     Common::StateVector ekfState,
                                     Common::SensorSnapshot sensorSnapshot) {
    if (simEnabled) {
        return currentState;
    }

    altWindow.push(ekfState.z);
    vzWindow.push(ekfState.vz);

    switch (currentState) {
        case Common::FlightState::BOOTUP:
            if (ekfState.vz > 1.0f) {
                return Common::FlightState::ASCENT;
            }
            break;
        case Common::FlightState::ASCENT:
            if (ekfState.vz < 0.0f && ekfState.z > 10.0f && altWindow.isDescending()) {
                return Common::FlightState::DESCENT;
            }
            if (ekfState.z > MaxAltitude) {
                MaxAltitude = ekfState.z;
            }
            break;
        case Common::FlightState::DESCENT:
            if (ekfState.z < 10.0f) {
                if (ekfState.z * 0.8f <= MaxAltitude
                    && sensorSnapshot.barometerData.altitude < MaxAltitude * 0.8f) {
                    return Common::FlightState::PAYLOAD_RELEASE;
                }
            }
            break;
        case Common::FlightState::PAYLOAD_RELEASE:
            accelWindow.push(std::sqrt(sensorSnapshot.imuData.acceleration.x * sensorSnapshot.imuData.acceleration.x
                + sensorSnapshot.imuData.acceleration.y * sensorSnapshot.imuData.acceleration.y
                + sensorSnapshot.imuData.acceleration.z * sensorSnapshot.imuData.acceleration.z));
            gyroWindow.push(std::sqrt(sensorSnapshot.imuData.gyroVel.x * sensorSnapshot.imuData.gyroVel.x
                + sensorSnapshot.imuData.gyroVel.y * sensorSnapshot.imuData.gyroVel.y
                + sensorSnapshot.imuData.gyroVel.z * sensorSnapshot.imuData.gyroVel.z));
            if (accelWindow.isStatic() && gyroWindow.isStatic()) {
                return Common::FlightState::RECOVERY;
            }
            break;
        case Common::FlightState::RECOVERY:
        case Common::FlightState::STANDBY:
        default:
            break;
    }

    return currentState;
}

}  // namespace MissionManager
