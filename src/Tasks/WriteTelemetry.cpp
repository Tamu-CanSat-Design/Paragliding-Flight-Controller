#include "TaskHandler.h"

void Tasks::WriteTelemetryTask(void* pvParameters) {
    (void)pvParameters;
    char telemetryBuf[1024]; // Buffer for telemetry data

    Common::SensorSnapshot localSnapshot = {};
    Common::StateVector localStateVector = {};
    Common::FlightState localFlightState = Common::FlightState::STANDBY;
    char localCommand[32] = {};
    
    Common::telemetry data;
    bool simEnabled = false;
    bool localCXFlag = false;
    while (true) {
        if(xSemaphoreTake(SharedData::gCXFlagMutex, pdMS_TO_TICKS(100)) == pdTRUE){
            localCXFlag = SharedData::gCXFlag;
            xSemaphoreGive(SharedData::gCXFlagMutex);
        }
        

        //Mutexes to safely read shared data
        if(xSemaphoreTake(SharedData::gSnapshotMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            localSnapshot = SharedData::gSnapshot;
            xSemaphoreGive(SharedData::gSnapshotMutex);
        }
        if(xSemaphoreTake(SharedData::gStateVectorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            localStateVector = SharedData::gStateVector;
            xSemaphoreGive(SharedData::gStateVectorMutex);
        }
        if(xSemaphoreTake(SharedData::gFlightStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            localFlightState = SharedData::gFlightState;
            xSemaphoreGive(SharedData::gFlightStateMutex);
        }
        if(xSemaphoreTake(SharedData::gCommandMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            snprintf(localCommand, sizeof(localCommand), "%s", SharedData::gCommand); // Copy command under mutex
            xSemaphoreGive(SharedData::gCommandMutex);
        }
        if(xSemaphoreTake(SharedData::gSimEnabledMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            simEnabled = SharedData::gSimEnabled;
            xSemaphoreGive(SharedData::gSimEnabledMutex);
        }
        //Task Notification Receiving

        
        char timeStr[16] = {0};
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", localSnapshot.timeData.hour, localSnapshot.timeData.minute, localSnapshot.timeData.second);


        strncpy(data.missionTime, timeStr, sizeof(data.missionTime) - 1);
        data.missionTime[sizeof(data.missionTime) - 1] = '\0';
        data.Mode = simEnabled ? 'S' : 'F'; // 'S' for simulation mode, 'F' for flight mode
        data.state = localFlightState;
        data.baroAltitude = localSnapshot.barometerData.altitude;
        data.temperature = localSnapshot.barometerData.temperature;
        data.pressure = localSnapshot.barometerData.pressure;
        data.voltage = localSnapshot.voltageData.voltage;
        data.current = localSnapshot.voltageData.current;
        data.imuData = localSnapshot.imuData;
        data.magnetometer = localSnapshot.magData;
        snprintf(data.GPSTime, sizeof(data.GPSTime), "%s", localSnapshot.gpsData.time);
        data.GPSLoc = {localSnapshot.gpsData.latitude, localSnapshot.gpsData.longitude, localSnapshot.gpsData.altitude};
        data.GPSSats = localSnapshot.gpsData.satellites;
        data.stateVector = localStateVector;

        Common::BuildPacket(data, telemetryBuf, sizeof(telemetryBuf));
        
        
        if(localCXFlag) {
            SharedData::commlink.WriteGroundRadio(telemetryBuf);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust delay as needed
    }

        
}
