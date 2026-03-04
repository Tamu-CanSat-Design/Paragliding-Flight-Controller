#include "TaskHandler.h"

void Tasks::WriteTelemetryTask(void* pvParameters) {
    (void)pvParameters;
    char telemetryBuf[512]; // Buffer for telemetry data (adjust size as needed)

    Common::SensorSnapshot localSnapshot;
    Common::StateVector localStateVector;
    Common::FlightState localFlightState;
    char localCommand[10];
    bool simEnabled;
    Common::telemetry data;
    while (true) {
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
            strncpy(localCommand, SharedData::gCommand, sizeof(localCommand));
            xSemaphoreGive(SharedData::gCommandMutex);
        }
        data.missionTime = millis() / 1000.0f; //
        data.Mode = simEnabled ? 'S' : 'N'; // 'S' for simulation mode, 'N' for normal mode
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
        

        SharedData::commlink.WriteGroundRadio(telemetryBuf);
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
    }

        
}
