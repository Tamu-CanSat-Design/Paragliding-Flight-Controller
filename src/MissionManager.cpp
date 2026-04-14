#include "MissionManager.h"

#include "SharedData.h"
#include "TaskHandler.h"

#include <cmath>
#include <cstring>
#include <cstdlib>

namespace {
    // void SetSharedSimEnabled(bool enabled) {
    //     if (SharedData::gSimEnabledMutex == nullptr) {
    //         return;
    //     }

    //     if (xSemaphoreTake(SharedData::gSimEnabledMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    //         SharedData::gSimEnabled = enabled;
    //         xSemaphoreGive(SharedData::gSimEnabledMutex);
    //     }
    // }

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
        static Common::Window<30> accelWindow;
        static Common::Window<30> gyroWindow;



    // void SIM() {
    //     const char* token = std::strchr(Command, ',');
    //     if (token == nullptr) {
    //         simEnabled = !simEnabled;
    //         simArmed = simEnabled;
    //         SetSharedSimEnabled(simEnabled);
    //         return;
    //     }

    //     ++token;
    //     if (std::strcmp(token, "ENABLE") == 0) {
    //         simArmed = true;
    //         simEnabled = false;
    //     } else if (std::strcmp(token, "ACTIVATE") == 0) {
    //         if (simArmed) {
    //             simEnabled = true;
    //         }
    //     } else if (std::strcmp(token, "DISABLE") == 0) {
    //         simEnabled = false;
    //         simArmed = false;
    //     }

    //     SetSharedSimEnabled(simEnabled);
    // }



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

    

  





    
    Common::FlightState CheckStateChange(Common::FlightState currentState, Common::StateVector ekfState, Common::SensorSnapshot sensorSnapshot){
        //Ascent Starts when velocity is above 1 m/s
        // if Inertial_Velocity > 1 m/s



        if(!simEnabled){
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
        else{
            //idk

            return currentState;
        }
        return currentState;
    }  
}// namespace MissionManager