#include <cstring>

#include "SharedData.h"

namespace{
    constexpr UBaseType_t BAROMETERQUEUEDEPTH = 10;
    constexpr UBaseType_t GPSQUEUEDEPTH = 10;
    constexpr UBaseType_t LIDARQUEUEDEPTH = 10;
    constexpr UBaseType_t I2C1CONTROLQUEUEDEPTH = 4;
    constexpr UBaseType_t IMUQUEUEDEPTH = 10;
    constexpr UBaseType_t MAGQUEUEDEPTH = 10;
    constexpr UBaseType_t VOLTAGEQUEUEDEPTH = 10;
    constexpr UBaseType_t COMMANDQUEUEDEPTH = 8;

    uint8_t gI2C1BarometerQueueStorage[BAROMETERQUEUEDEPTH * sizeof(Common::BarometerData)];
    uint8_t gI2C1GPSQueueStorage[GPSQUEUEDEPTH * sizeof(Common::GPSData)];
    uint8_t gI2C1LidarQueueStorage[LIDARQUEUEDEPTH * sizeof(Common::LidarData)];
    uint8_t gI2C1ControlQueueStorage[I2C1CONTROLQUEUEDEPTH * sizeof(Common::I2C1ControlCommand)];
    uint8_t gI2C1ControlResultQueueStorage[I2C1CONTROLQUEUEDEPTH * sizeof(Common::I2C1ControlResult)];
    uint8_t gI2C2IMUQueueStorage[IMUQUEUEDEPTH * sizeof(Common::IMUData)];
    uint8_t gI2C2MagQueueStorage[MAGQUEUEDEPTH * sizeof(Common::MagData)];
    uint8_t gI2C2VoltageQueueStorage[VOLTAGEQUEUEDEPTH * sizeof(Common::VoltageData)];
    uint8_t gCommandQueueStorage[COMMANDQUEUEDEPTH * Common::COMMAND_BUFFER_LENGTH];
}


namespace SharedData {

    Commlink commlink;

    // Storage for the shared sensor snapshot.
    Common::SensorSnapshot gSnapshot;
    SemaphoreHandle_t gSnapshotMutex = nullptr;
    static StaticSemaphore_t gSnapshotMutexBuffer;


    Common::FlightState gFlightState;
    SemaphoreHandle_t gFlightStateMutex = nullptr;
    static StaticSemaphore_t gFlightStateMutexBuffer;


    bool gSimEnabled = false;
    SemaphoreHandle_t gSimEnabledMutex = nullptr;
    static StaticSemaphore_t gSimEnabledMutexBuffer;

    bool gCXFlag = false;
    SemaphoreHandle_t gCXFlagMutex = nullptr;
    static StaticSemaphore_t gCXFlagMutexBuffer;

    Common::StateVector gStateVector;
    SemaphoreHandle_t gStateVectorMutex = nullptr;
    static StaticSemaphore_t gStateVectorMutexBuffer;

    char gCommand[Common::COMMAND_BUFFER_LENGTH] = {};
    SemaphoreHandle_t gCommandMutex = nullptr;
    static StaticSemaphore_t gCommandMutexBuffer;

    float gSimPressureHpa = 0.0f;
    SemaphoreHandle_t gSimPressureMutex = nullptr;
    static StaticSemaphore_t gSimPressureMutexBuffer;

//Queues
    static StaticQueue_t gI2C1BarometerQueueBuffer;
    static StaticQueue_t gI2C1GPSQueueBuffer;
    static StaticQueue_t gI2C1LidarQueueBuffer;
    static StaticQueue_t gI2C1ControlQueueBuffer;
    static StaticQueue_t gI2C1ControlResultQueueBuffer;
    QueueHandle_t gI2C1BarometerQueue = nullptr;
    QueueHandle_t gI2C1GPSQueue = nullptr;
    QueueHandle_t gI2C1LidarQueue = nullptr;
    QueueHandle_t gI2C1ControlQueue = nullptr;
    QueueHandle_t gI2C1ControlResultQueue = nullptr;
    
    static StaticQueue_t gI2C2IMUQueueBuffer;
    static StaticQueue_t gI2C2MagQueueBuffer;
    static StaticQueue_t gI2C2VoltageQueueBuffer;
    QueueHandle_t gI2C2IMUQueue = nullptr;
    QueueHandle_t gI2C2MagQueue = nullptr;
    QueueHandle_t gI2C2VoltageQueue = nullptr;

    static StaticQueue_t gCommandQueueBuffer;
    QueueHandle_t gCommandQueue = nullptr;
    



    void InitializeThreadSafeData() {
        gSnapshotMutex = xSemaphoreCreateMutexStatic(&gSnapshotMutexBuffer);
        gFlightStateMutex = xSemaphoreCreateMutexStatic(&gFlightStateMutexBuffer);
        gSimEnabledMutex = xSemaphoreCreateMutexStatic(&gSimEnabledMutexBuffer);
        gStateVectorMutex = xSemaphoreCreateMutexStatic(&gStateVectorMutexBuffer);
        gCommandMutex = xSemaphoreCreateMutexStatic(&gCommandMutexBuffer);
        gCXFlagMutex = xSemaphoreCreateMutexStatic(&gCXFlagMutexBuffer);
        gSimPressureMutex = xSemaphoreCreateMutexStatic(&gSimPressureMutexBuffer);

        gI2C1BarometerQueue = xQueueCreateStatic(BAROMETERQUEUEDEPTH, sizeof(Common::BarometerData), gI2C1BarometerQueueStorage, &gI2C1BarometerQueueBuffer);
        gI2C1GPSQueue = xQueueCreateStatic(GPSQUEUEDEPTH, sizeof(Common::GPSData), gI2C1GPSQueueStorage, &gI2C1GPSQueueBuffer);
        gI2C1LidarQueue = xQueueCreateStatic(LIDARQUEUEDEPTH, sizeof(Common::LidarData), gI2C1LidarQueueStorage, &gI2C1LidarQueueBuffer);
        gI2C1ControlQueue = xQueueCreateStatic(I2C1CONTROLQUEUEDEPTH, sizeof(Common::I2C1ControlCommand), gI2C1ControlQueueStorage, &gI2C1ControlQueueBuffer);
        gI2C1ControlResultQueue = xQueueCreateStatic(I2C1CONTROLQUEUEDEPTH, sizeof(Common::I2C1ControlResult), gI2C1ControlResultQueueStorage, &gI2C1ControlResultQueueBuffer);
        gI2C2IMUQueue = xQueueCreateStatic(IMUQUEUEDEPTH, sizeof(Common::IMUData), gI2C2IMUQueueStorage, &gI2C2IMUQueueBuffer);
        gI2C2MagQueue = xQueueCreateStatic(MAGQUEUEDEPTH, sizeof(Common::MagData), gI2C2MagQueueStorage, &gI2C2MagQueueBuffer);
        gI2C2VoltageQueue = xQueueCreateStatic(VOLTAGEQUEUEDEPTH, sizeof(Common::VoltageData), gI2C2VoltageQueueStorage, &gI2C2VoltageQueueBuffer);
        gCommandQueue = xQueueCreateStatic(COMMANDQUEUEDEPTH, Common::COMMAND_BUFFER_LENGTH, gCommandQueueStorage, &gCommandQueueBuffer);

        gFlightState = Common::FlightState::BOOTUP;
        gSimEnabled = false;
        gCXFlag = false;
        memset(gCommand, 0, sizeof(gCommand));
    }
}
