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

    static Window<float, 10> altWindow;
    static Window<float, 10> vzWindow;
    static Window<float, 10> accelWindow;
    static Window<float, 10> gyroWindow;

    void ST() {
        // Implement ST command logic
        char* token = strrchr(Command, ',');
        if (!token) return;
        token++;
        if (strncmp(token, "GPS", 3) == 0) {
            //mission time = gpstime
        }
        else {
            // mission time == token
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
        float altitude = 44330.0f * (1.0f - powf(pressure / /* ground pressure */, 0.1903f));
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
        EGRELFlag = true;
    }

    void LDAR() {
        LDARFlag = !LDARFlag;
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
                    if(ekfState.vz < 0.0f && sensorSnapshot.barometerData.altitude > 10.0f && altWindow.isDescending()) { // Using a sliding window, make sure altitude starts decreasing
                        return Common::FlightState::DESCENT;
                    }
                    if (sensorSnapshot.barometerData.altitude > MaxAltitude) {
                        MaxAltitude = sensorSnapshot.barometerData.altitude;
                    }
                    break;
                case Common::FlightState::DESCENT:
                    if(sensorSnapshot.barometerData.altitude < 10.0f) { //store the max altitude, if 80% of max altitude is reached, trigger payload release
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
        }
    }
    

};