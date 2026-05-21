#include "MissionManager.h"

#include "SharedData.h"
#include "TaskHandler.h"

#include <cmath>
#include <cstring>
#include <cstdlib>

namespace {

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
    bool localCXFlag = false;
    bool LDARFlag = false;
    bool EGRELFlag = false;
    bool AUDFlag = false;
    bool localEKFFlag = false;
    bool barometerMode = false;

    bool StandbyStartedFlag = false;
    bool AscentStartedFlag = false;
    bool DescentStartedFlag = false;
    bool PayloadReleaseStartedFlag = false;
    bool RecoveryStartedFlag = false;
    float MaxAltitude = 0.0f;

    Common::Window<10> altWindow;
    Common::Window<10> vzWindow;
    Common::Window<30> accelWindow;
    Common::Window<30> gyroWindow;
    
    Common::Window<20> baroAltWindow;
    Common::Window<2> simulationWindow;
    


    void SIM() {
        simEnabled = !simEnabled;
        simArmed = simEnabled;

        if (SharedData::gSimEnabledMutex != nullptr) {
            if (xSemaphoreTake(SharedData::gSimEnabledMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                SharedData::gSimEnabled = simEnabled;
                xSemaphoreGive(SharedData::gSimEnabledMutex);
            }
        }
    }

    void SetBarometerMode(bool enabled) {
        BarometerMode = enabled;
    }

    void CAL() {
        float pressureHpa = 0.0f;
        if (!TryGetCurrentBarometerPressure(&pressureHpa) || SharedData::gBarometerCalibrationQueue == nullptr) {
            return;
        }

        if (xQueueOverwrite(SharedData::gBarometerCalibrationQueue, &pressureHpa) != pdPASS) {
            return;
        }

        MaxAltitude = 0.0f;
        altWindow.clear();
        vzWindow.clear();
        accelWindow.clear();
        gyroWindow.clear();
    }


    
    Common::FlightState CheckStateChange(Common::FlightState currentState, Common::StateVector ekfState, Common::SensorSnapshot sensorSnapshot){
        //Ascent Starts when velocity is above 1 m/s
        // if Inertial_Velocity > 1 m/s


        if(simEnabled) 
        {
            switch(currentState){
                case Common::FlightState::STANDBY:
                    if(simulationWindow.size() > 1 && simulationWindow.isDescending()) {
                        return Common::FlightState::ASCENT;
                    }
                    break;
                case Common::FlightState::ASCENT:
                    if(simulationWindow.isAscending()) {
                        return Common::FlightState::DESCENT;
                    }
                    if (simulationWindow.getLatest()> MaxAltitude) {
                        MaxAltitude = simulationWindow.getLatest();
                    }
                    break;
                case Common::FlightState::DESCENT:
                    if(simulationWindow.getLatest() < (0.8f * MaxAltitude)){
                        return Common::FlightState::PAYLOAD_RELEASE;
                    }
                    break;
                case Common::FlightState::PAYLOAD_RELEASE:
                    if(simulationWindow.isStatic()) {
                        return Common::FlightState::RECOVERY;
                    }
            }
        }
        // else if(barometerMode)
        // {

        // }
        else 
        {
            // Logic for checking state changes
            altWindow.push(ekfState.z);
            vzWindow.push(ekfState.vz);
            switch(currentState){
                case Common::FlightState::STANDBY:
                    if(ekfState.vz > 1.0f) {
                        return Common::FlightState::ASCENT;
                    }
                    break;
                case Common::FlightState::ASCENT:
                    if(ekfState.vz < 0.0f && ekfState.z > 10.0f && altWindow.isDescending()) { // Using a sliding window, make sure altitude starts decreasing
                        return Common::FlightState::DESCENT;
                    }
                    if (ekfState.z > MaxAltitude) {
                        MaxAltitude = ekfState.z;
                    }
                    break;
                case Common::FlightState::DESCENT:
                    if(ekfState.z < 10.0f) { //store the max altitude, if 80% of max altitude is reached, trigger payload release
                        //check with barometer sliding window and with extended kalman filter altitude (ekfstate.z)
                        if (ekfState.z * 0.8 <= MaxAltitude && sensorSnapshot.barometerData.altitude < MaxAltitude * 0.8) {
                            return Common::FlightState::PAYLOAD_RELEASE;
                        }
                    }
                    break;
                case Common::FlightState::PAYLOAD_RELEASE:
                    // Check if sensor data is static (e.g., low acceleration, low gyro rates) for a certain duration to confirm landing
                    accelWindow.push(sqrtf(sensorSnapshot.imuData.acceleration.x * sensorSnapshot.imuData.acceleration.x +
                                            sensorSnapshot.imuData.acceleration.y * sensorSnapshot.imuData.acceleration.y +
                                            sensorSnapshot.imuData.acceleration.z * sensorSnapshot.imuData.acceleration.z));
                    gyroWindow.push(sqrtf(sensorSnapshot.imuData.gyroVel.x * sensorSnapshot.imuData.gyroVel.x +
                                        sensorSnapshot.imuData.gyroVel.y * sensorSnapshot.imuData.gyroVel.y +
                                        sensorSnapshot.imuData.gyroVel.z * sensorSnapshot.imuData.gyroVel.z));
                    if (accelWindow.isStatic() && gyroWindow.isStatic()) {
                        return Common::FlightState::RECOVERY;
                    }
                    break;
                case Common::FlightState::RECOVERY:
                    // Final state, may not transition out of this
                    break;
                default:
                    // Handle unexpected state
                    break;
            }
        }
        return currentState;
    }  
}// namespace MissionManager
