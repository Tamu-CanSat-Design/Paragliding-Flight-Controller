#include <cstring>
#include "SharedData.h"

namespace{
    constexpr UBaseType_t BAROMETERQUEUEDEPTH = 20;
    constexpr UBaseType_t GPSQUEUEDEPTH = 10;
    constexpr UBaseType_t LIDARQUEUEDEPTH = 20;
    constexpr UBaseType_t IMUQUEUEDEPTH = 50;
    constexpr UBaseType_t MAGQUEUEDEPTH = 30;
    constexpr UBaseType_t VOLTAGEQUEUEDEPTH = 10;
    constexpr UBaseType_t COMMANDQUEUEDEPTH = 8;

    uint8_t gI2C1BarometerQueueStorage[BAROMETERQUEUEDEPTH * sizeof(Common::BarometerData)];
    uint8_t gI2C1GPSQueueStorage[GPSQUEUEDEPTH * sizeof(Common::GPSData)];
    uint8_t gI2C1LidarQueueStorage[LIDARQUEUEDEPTH * sizeof(Common::LidarData)];
    uint8_t gI2C2IMUQueueStorage[IMUQUEUEDEPTH * sizeof(Common::IMUBatch)];
    uint8_t gI2C2MagQueueStorage[MAGQUEUEDEPTH * sizeof(Common::MagData)];
    uint8_t gI2C1VoltageQueueStorage[VOLTAGEQUEUEDEPTH * sizeof(Common::VoltageData)];
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

    float gReferencePressureHpa = 1013.25f;
    SemaphoreHandle_t gReferencePressureMutex = nullptr;
    static StaticSemaphore_t gReferencePressureMutexBuffer;

    Common::StateVector gStateVector;
    SemaphoreHandle_t gStateVectorMutex = nullptr;
    static StaticSemaphore_t gStateVectorMutexBuffer;

    float gRollAngleSetpoint = 0.0f;
    SemaphoreHandle_t gRollAngleSetpointMutex = nullptr;
    static StaticSemaphore_t gRollAngleSetpointMutexBuffer;

    float gRollAngularVelocity = 0.0f;
    SemaphoreHandle_t gRollAngularVelocityMutex = nullptr;
    static StaticSemaphore_t gRollAngularVelocityMutexBuffer;

    char gCommand[Common::COMMAND_BUFFER_LENGTH] = {};
    SemaphoreHandle_t gCommandMutex = nullptr;
    static StaticSemaphore_t gCommandMutexBuffer;

//Queues
    static StaticQueue_t gI2C1BarometerQueueBuffer;
    static StaticQueue_t gI2C1GPSQueueBuffer;
    static StaticQueue_t gI2C1LidarQueueBuffer;
    QueueHandle_t gI2C1BarometerQueue = nullptr;
    QueueHandle_t gI2C1GPSQueue = nullptr;
    QueueHandle_t gI2C1LidarQueue = nullptr;
    
    static StaticQueue_t gI2C2IMUQueueBuffer;
    static StaticQueue_t gI2C2MagQueueBuffer;
    static StaticQueue_t gI2C1VoltageQueueBuffer;
    QueueHandle_t gI2C2IMUQueue = nullptr;
    QueueHandle_t gI2C2MagQueue = nullptr;
    QueueHandle_t gI2C1VoltageQueue = nullptr;

    static StaticQueue_t gCommandQueueBuffer;
    QueueHandle_t gCommandQueue = nullptr;
    


    void InitializeThreadSafeData() {
        gSnapshotMutex = xSemaphoreCreateMutexStatic(&gSnapshotMutexBuffer);
        gFlightStateMutex = xSemaphoreCreateMutexStatic(&gFlightStateMutexBuffer);
        gSimEnabledMutex = xSemaphoreCreateMutexStatic(&gSimEnabledMutexBuffer);
        gStateVectorMutex = xSemaphoreCreateMutexStatic(&gStateVectorMutexBuffer);
        gCommandMutex = xSemaphoreCreateMutexStatic(&gCommandMutexBuffer);
        gCXFlagMutex = xSemaphoreCreateMutexStatic(&gCXFlagMutexBuffer);
        gReferencePressureMutex = xSemaphoreCreateMutexStatic(&gReferencePressureMutexBuffer);
        gRollAngleSetpointMutex = xSemaphoreCreateMutexStatic(&gRollAngleSetpointMutexBuffer);
        gRollAngularVelocityMutex = xSemaphoreCreateMutexStatic(&gRollAngularVelocityMutexBuffer);
        gI2C1BarometerQueue = xQueueCreateStatic(BAROMETERQUEUEDEPTH, sizeof(Common::BarometerData), gI2C1BarometerQueueStorage, &gI2C1BarometerQueueBuffer);
        gI2C1GPSQueue = xQueueCreateStatic(GPSQUEUEDEPTH, sizeof(Common::GPSData), gI2C1GPSQueueStorage, &gI2C1GPSQueueBuffer);
        gI2C1LidarQueue = xQueueCreateStatic(LIDARQUEUEDEPTH, sizeof(Common::LidarData), gI2C1LidarQueueStorage, &gI2C1LidarQueueBuffer);
        // IMU queue now carries FIFO-drained batches instead of one sample per send.
        gI2C2IMUQueue = xQueueCreateStatic(IMUQUEUEDEPTH, sizeof(Common::IMUBatch), gI2C2IMUQueueStorage, &gI2C2IMUQueueBuffer);
        gI2C2MagQueue = xQueueCreateStatic(MAGQUEUEDEPTH, sizeof(Common::MagData), gI2C2MagQueueStorage, &gI2C2MagQueueBuffer);
        gI2C1VoltageQueue = xQueueCreateStatic(VOLTAGEQUEUEDEPTH, sizeof(Common::VoltageData), gI2C1VoltageQueueStorage, &gI2C1VoltageQueueBuffer);
        gCommandQueue = xQueueCreateStatic(COMMANDQUEUEDEPTH, Common::COMMAND_BUFFER_LENGTH, gCommandQueueStorage, &gCommandQueueBuffer);

        gFlightState = Common::FlightState::STANDBY;
        gSimEnabled = false;
        gCXFlag = false;
        gReferencePressureHpa = 1013.25f;
        memset(gCommand, 0, sizeof(gCommand));

        if(!gSnapshotMutex || !gFlightStateMutex || !gSimEnabledMutex || !gStateVectorMutex || !gCommandMutex || !gCXFlagMutex || !gReferencePressureMutex || !gRollAngleSetpointMutex || !gRollAngularVelocityMutex ||
           !gI2C1BarometerQueue || !gI2C1GPSQueue || !gI2C1LidarQueue || !gI2C2IMUQueue || !gI2C2MagQueue || !gI2C1VoltageQueue || !gCommandQueue) {
            // Handle error: insufficient resources to create mutexes or queues
            Serial.println("Error initializing shared data mutexes or queues");
            while (true) {} // Halt execution
        }
    }
}
