#include "TaskHandler.h"

void Tasks::WriteTelemetryTask(void* pvParameters) {
    (void)pvParameters;
    char telemetryBuf[1024]; // Buffer for telemetry data

    Common::SensorSnapshot localSnapshot = {};
    Common::StateVector localStateVector = {};
    Common::FlightState localFlightState = Common::FlightState::STANDBY;
    char localCommand[32] = {};
    bool simEnabled = false;
    Common::telemetry data;

    bool localCXFlag = false;
    while (true) {
        if(ulTaskNotifyTake(pdFALSE, 0) % 2 == 1){
            localCXFlag = true;
        }
        else{
            localCXFlag = false;
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
            strncpy(localCommand, SharedData::gCommand, sizeof(localCommand) - 1);
            localCommand[sizeof(localCommand) - 1] = '\0'; // Copy command under mutex
            xSemaphoreGive(SharedData::gCommandMutex);
        }
        if(xSemaphoreTake(SharedData::gSimEnabledMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            simEnabled = SharedData::gSimEnabled;
            xSemaphoreGive(SharedData::gSimEnabledMutex);
        }
        //Task Notification Receiving




        data.missionTime = static_cast<float>(millis()) * 0.001f;
        data.Mode = simEnabled ? 'S' : 'F'; // 'S' for simulation mode, 'N' for normal mode
        data.state = localFlightState;
        data.baroAltitude = localSnapshot.barometerData.altitude;
        data.temperature = localSnapshot.barometerData.temperature;
        data.pressure = localSnapshot.barometerData.pressure;
        data.voltage = localSnapshot.voltageData.voltage;
        data.current = localSnapshot.voltageData.current;
        data.imuData = localSnapshot.imuData;
        data.magnetometer = localSnapshot.magData;
        data.GPSTime = localSnapshot.gpsData.time;
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
