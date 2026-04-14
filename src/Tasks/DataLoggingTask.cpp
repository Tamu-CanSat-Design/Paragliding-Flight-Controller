#include "TaskHandler.h"
#include <SD.h>

namespace{
    const char* CSV_HEADER = "ID,Time,PacketCount,Mode,State,BaroAlt,Temp,Pressure,"
            "Voltage,Current,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,GPSTime,"
            "GPSLat,GPSLon,GPSAlt,GPSSats,CMDEcho,MagX,MagY,MagZ,PosX:"
            "PosY,PosZ,VelX,VelY,VelZ,QuatW,QuatX,QuatY,QuatZ,AccelBias,GyroBias,EKFStatus\n";
}


void Tasks::DataLoggingTask(void* pvParameters) {
    (void)pvParameters;

    // Initialize SD card (runs once)
    if (!SD.begin(Common::SD_PIN)) {
        Serial.println("[DataLog] SD init failed — task suspending");
        vTaskSuspend(nullptr);          // nothing we can do without the card
    }

    // Write header if file is new / empty
    {
        File f = SD.open(Common::LOG_FILE, FILE_WRITE);
        if (f) {
            if (f.size() == 0) {
                f.print(CSV_HEADER);
            }
            f.close();
        }
    }

    Common::SensorSnapshot localSensorSnapshot = {};
    Common::StateVector localStateVector = {};
    Common::FlightState localFlightState = Common::FlightState::STANDBY;
    bool simEnabled = false;
    char lineBuf[1024]; // Buffer for one line of CSV

    while (true) {
        // 1) Copy snapshot under mutex (fast)
        if (xSemaphoreTake(SharedData::gSnapshotMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            localSensorSnapshot = SharedData::gSnapshot;
            xSemaphoreGive(SharedData::gSnapshotMutex);
        }

        // 2) Copy flight state under its own mutex
        if (xSemaphoreTake(SharedData::gFlightStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            localFlightState = SharedData::gFlightState;
            xSemaphoreGive(SharedData::gFlightStateMutex);
        }

        // 3) Copy state vector under its own mutex
        if (xSemaphoreTake(SharedData::gStateVectorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            localStateVector = SharedData::gStateVector;
            xSemaphoreGive(SharedData::gStateVectorMutex);
        }
        
        if(xSemaphoreTake(SharedData::gSimEnabledMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            simEnabled = SharedData::gSimEnabled;
            xSemaphoreGive(SharedData::gSimEnabledMutex);
        }

        // 4) Format the CSV line (no mutex held)
        Common::telemetry dataToLog = {};
        dataToLog.missionTime = static_cast<float>(millis()) * 0.001f;
        dataToLog.Mode = simEnabled ? 'S' : 'F'; // 'S' for simulation mode, 'F' for flight mode
        dataToLog.state = localFlightState;
        dataToLog.baroAltitude = localSensorSnapshot.barometerData.altitude;
        dataToLog.temperature = localSensorSnapshot.barometerData.temperature;
        dataToLog.pressure = localSensorSnapshot.barometerData.pressure;
        dataToLog.voltage = localSensorSnapshot.voltageData.voltage;
        dataToLog.current = localSensorSnapshot.voltageData.current;
        dataToLog.imuData = localSensorSnapshot.imuData;
        dataToLog.magnetometer = localSensorSnapshot.magData;
        dataToLog.GPSTime = localSensorSnapshot.gpsData.time;
        dataToLog.GPSLoc = {localSensorSnapshot.gpsData.latitude, localSensorSnapshot.gpsData.longitude, localSensorSnapshot.gpsData.altitude};
        dataToLog.GPSSats = localSensorSnapshot.gpsData.satellites;
        dataToLog.stateVector = localStateVector;
        int len = Common::BuildPacket(dataToLog, lineBuf, sizeof(lineBuf));

        // 4) Append to SD card (slow I/O, no mutex held)
        if (len > 0) {
            File f = SD.open(Common::LOG_FILE, FILE_WRITE);
            if (f) {
                f.write((const uint8_t*)lineBuf, len);
                f.close();                              // flushes data to card
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));   // Log at ~10 Hz
    }
}