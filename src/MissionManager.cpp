#include "MissionManager.h"
#include "SharedData.h"



namespace MissionManager {


    char Command[32];

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
    float MaxAltitude = 0.0;

    Common::Window<float, 10> altWindow;
    Common::Window<float, 10> vzWindow;
    Common::Window<float, 10> accelWindow;
    Common::Window<float, 10> gyroWindow;

    void ST() {
        char* token = strrchr(Command, ',');
        if (!token) return;
        token++;
        if (strncmp(token, "GPS", 3) == 0) {
            if (xSemaphoreTake(SharedData::gSnapshotMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (xSemaphoreTake(SharedData::gMissionTimeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    SharedData::missionTime = SharedData::gSnapshot.gpsData.time;
                    xSemaphoreGive(SharedData::gMissionTimeMutex);
                }
                xSemaphoreGive(SharedData::gSnapshotMutex);
            }
        }
        else {
            int hours = 0;
            int minutes = 0;
            int seconds = 0;

            if (sscanf(token, "%d:%d:%d", &hours, &minutes, &seconds) == 3 &&
                hours >= 0 && hours < 24 &&
                minutes >= 0 && minutes < 60 &&
                seconds >= 0 && seconds < 60) {
                if (xSemaphoreTake(SharedData::gMissionTimeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    SharedData::missionTime = static_cast<float>(hours * 3600 + minutes * 60 + seconds);
                    xSemaphoreGive(SharedData::gMissionTimeMutex);
                }
            }
        }
    }

    void SIM() {
        char* token = strrchr(Command, ',');
        if (!token) return;
        token++;
        if (strncmp(token, "ENABLE", 6) == 0) { simArmed = true; simEnabled = false; }
        else if (strncmp(token, "ACTIVATE", 8) == 0) { if (simArmed) simEnabled = true; }
        else if (strncmp(token, "DISABLE", 7) == 0) { simEnabled = false; simArmed = false; }
    }

    void SIMP() {
        // Implement SIMP command logic
        // Turn pressure into an altitude
        if (!simEnabled) return;
        char* token = strrchr(Command, ',');
        if (!token) return;
        token++;
        float pressure = strtof(token, nullptr) / 100.0f;
        if (pressure <= 0.0f) return;
        float altitude = 44330.0f * (1.0f - powf(pressure / 101, 0.1903f));
        altWindow.push(altitude);
    }

    void CAL() {
        // Implement CAL command logic
        //ground pressure
        MaxAltitude = 0.0f;
        altWindow.clear();
        vzWindow.clear();
        accelWindow.clear();
        gyroWindow.clear();
    }

    void EGREL() {
        EGRELFlag = !EGRELFlag;
    }

    void LDAR() {
        LDARFlag = !LDARFlag;
    }

    
    bool isDescending(const Common::Window<float, 10>& window) {
        int count = window.size();
        if(count < 2) return false;
        int down = 0;
        for(int i = 1; i < count; i++){
            float prev = window[i - 1];
            float curr = window[i];
            if(curr < prev) down++;
        }
        return down > (count - 1) / 2;
    }

    bool isAscending(const Common::Window<float, 10>& window) {
        int count = window.size();
        if(count < 2) return false;
        int up = 0;
        for(int i = 1; i < count; i++){
            float prev = window[i - 1];
            float curr = window[i];
            if(curr > prev) up++;
        }
        return up > (count - 1) / 2;
    }

    bool isStatic(const Common::Window<float, 10>& window) {
        int count = window.size();
        if(count < 2) return false;
        int flat = 0;
        for(int i = 1; i < count; i++){
            float prev = window[i - 1];
            float curr = window[i];
            if(std::fabs(curr - prev) < 0.1f) flat++;
        }
        return flat > (count - 1) / 2;
    }
  





    
    Common::FlightState CheckStateChange(Common::FlightState currentState, Common::StateVector ekfState, Common::SensorSnapshot sensorSnapshot){
        //Ascent Starts when velocity is above 1 m/s
        // if Inertial_Velocity > 1 m/s



        if(!simEnabled){
            // Logic for checking state changes
            altWindow.push(sensorSnapshot.barometerData.altitude);
            vzWindow.push(ekfState.vz);
            switch(currentState){
                case Common::FlightState::BOOTUP:
                    if(ekfState.vz > 1.0f) {
                        return Common::FlightState::ASCENT;
                    }
                    break;
                case Common::FlightState::ASCENT:
                    if(ekfState.vz < 0.0f && sensorSnapshot.barometerData.altitude > 10.0f && isDescending(altWindow)) { // Using a sliding window, make sure altitude starts decreasing
                        return Common::FlightState::DESCENT;
                    }
                    if (sensorSnapshot.barometerData.altitude > MaxAltitude) {
                        MaxAltitude = sensorSnapshot.barometerData.altitude;
                    }
                    break;
                case Common::FlightState::DESCENT:
                    if(sensorSnapshot.barometerData.altitude < 10.0f) { //store the max altitude, if 80% of max altitude is reached, trigger payload release
                        //check with barometer sliding window and with extended kalman filter altitude (ekfstate.z)
                        if (ekfState.z * 0.8 <= MaxAltitude) {
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
                    if (isStatic(accelWindow) && isStatic(gyroWindow)) {
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
            altWindow.push(sensorSnapshot.barometerData.altitude);
            vzWindow.push(ekfState.vz);
            switch(currentState){
                case Common::FlightState::BOOTUP:
                    if(ekfState.vz > 1.0f) {
                        return Common::FlightState::ASCENT;
                    }
                    break;
                case Common::FlightState::ASCENT:
                    if(ekfState.vz < 0.0f && sensorSnapshot.barometerData.altitude > 10.0f && isDescending(altWindow)) { // Using a sliding window, make sure altitude starts decreasing
                        return Common::FlightState::DESCENT;
                    }
                    if (sensorSnapshot.barometerData.altitude > MaxAltitude) {
                        MaxAltitude = sensorSnapshot.barometerData.altitude;
                    }
                    break;
                case Common::FlightState::DESCENT:
                    if(sensorSnapshot.barometerData.altitude < 10.0f) { //store the max altitude, if 80% of max altitude is reached, trigger payload release
                        //check with barometer sliding window and with extended kalman filter altitude (ekfstate.z)
                        if (ekfState.z * 0.8 <= MaxAltitude) {
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
                    if (isStatic(accelWindow) && isStatic(gyroWindow)) {
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
    }
    

};