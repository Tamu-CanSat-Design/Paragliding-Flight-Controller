#include <cstring>
#include "SharedData.h"

namespace {
    constexpr UBaseType_t BAROMETERQUEUEDEPTH = 20;
    constexpr UBaseType_t GPSQUEUEDEPTH = 10;
    constexpr UBaseType_t LIDARQUEUEDEPTH = 20;
    constexpr UBaseType_t IMUQUEUEDEPTH = 50;
    constexpr UBaseType_t MAGQUEUEDEPTH = 30;
    constexpr UBaseType_t VOLTAGEQUEUEDEPTH = 10;
    constexpr UBaseType_t COMMANDQUEUEDEPTH = 8;
    constexpr UBaseType_t SETTIMEQUEUEDEPTH = 3;
    constexpr UBaseType_t BAROMETERCALIBRATIONQUEUEDEPTH = 3;

    uint8_t gI2C1BarometerQueueStorage[BAROMETERQUEUEDEPTH * sizeof(Common::BarometerData)];
    uint8_t gI2C1GPSQueueStorage[GPSQUEUEDEPTH * sizeof(Common::GPSData)];
    uint8_t gI2C1LidarQueueStorage[LIDARQUEUEDEPTH * sizeof(Common::LidarData)];
    uint8_t gI2C2IMUQueueStorage[IMUQUEUEDEPTH * sizeof(Common::IMUBatch)];
    uint8_t gI2C2MagQueueStorage[MAGQUEUEDEPTH * sizeof(Common::MagData)];
    uint8_t gI2C1VoltageQueueStorage[VOLTAGEQUEUEDEPTH * sizeof(Common::VoltageData)];
    uint8_t gCommandQueueStorage[COMMANDQUEUEDEPTH * Common::COMMAND_BUFFER_LENGTH];
    uint8_t gSetTimeQueueStorage[SETTIMEQUEUEDEPTH * Common::TIME_STRING_LENGTH];
    uint8_t gBarometerCalibrationQueueStorage[BAROMETERCALIBRATIONQUEUEDEPTH * sizeof(float)];

    StaticSemaphore_t gSnapshotMutexBuffer;
    StaticSemaphore_t gFlightStateMutexBuffer;
    StaticSemaphore_t gSimEnabledMutexBuffer;
    StaticSemaphore_t gCXFlagMutexBuffer;
    StaticSemaphore_t gKalmanFlagMutexBuffer;
    StaticSemaphore_t gStateVectorMutexBuffer;
    StaticSemaphore_t gRollAngleSetpointMutexBuffer;
    StaticSemaphore_t gRollAngularVelocityMutexBuffer;
    StaticSemaphore_t gCommandMutexBuffer;

    StaticQueue_t gI2C1BarometerQueueBuffer;
    StaticQueue_t gI2C1GPSQueueBuffer;
    StaticQueue_t gI2C1LidarQueueBuffer;
    StaticQueue_t gI2C2IMUQueueBuffer;
    StaticQueue_t gI2C2MagQueueBuffer;
    StaticQueue_t gI2C1VoltageQueueBuffer;
    StaticQueue_t gCommandQueueBuffer;
    StaticQueue_t gSetTimeQueueBuffer;
    StaticQueue_t gBarometerCalibrationQueueBuffer;
}

namespace SharedData {

    Common::SensorSnapshot gSnapshot;
    Common::FlightState gFlightState;
    bool gSimEnabled = false;
    bool gCXFlag = false;
    bool gKalmanFlag = false;
    Common::StateVector gStateVector;
    float gRollAngleSetpoint = 0.0f;
    float gRollAngularVelocity = 0.0f;
    char gCommand[Common::COMMAND_BUFFER_LENGTH] = {};

    SemaphoreHandle_t gSnapshotMutex = nullptr;
    SemaphoreHandle_t gFlightStateMutex = nullptr;
    SemaphoreHandle_t gSimEnabledMutex = nullptr;
    SemaphoreHandle_t gCXFlagMutex = nullptr;
    SemaphoreHandle_t gKalmanFlagMutex = nullptr;
    SemaphoreHandle_t gStateVectorMutex = nullptr;
    SemaphoreHandle_t gRollAngleSetpointMutex = nullptr;
    SemaphoreHandle_t gRollAngularVelocityMutex = nullptr;
    SemaphoreHandle_t gCommandMutex = nullptr;

    QueueHandle_t gI2C1BarometerQueue = nullptr;
    QueueHandle_t gI2C1GPSQueue = nullptr;
    QueueHandle_t gI2C1LidarQueue = nullptr;
    QueueHandle_t gI2C2IMUQueue = nullptr;
    QueueHandle_t gI2C2MagQueue = nullptr;
    QueueHandle_t gI2C1VoltageQueue = nullptr;
    QueueHandle_t gCommandQueue = nullptr;
    QueueHandle_t gSetTimeQueue = nullptr;
    QueueHandle_t gBarometerCalibrationQueue = nullptr;

namespace {
    void CreateMutexes() {
        gSnapshotMutex = xSemaphoreCreateMutexStatic(&gSnapshotMutexBuffer);
        gFlightStateMutex = xSemaphoreCreateMutexStatic(&gFlightStateMutexBuffer);
        gSimEnabledMutex = xSemaphoreCreateMutexStatic(&gSimEnabledMutexBuffer);
        gStateVectorMutex = xSemaphoreCreateMutexStatic(&gStateVectorMutexBuffer);
        gCommandMutex = xSemaphoreCreateMutexStatic(&gCommandMutexBuffer);
        gCXFlagMutex = xSemaphoreCreateMutexStatic(&gCXFlagMutexBuffer);
        gKalmanFlagMutex = xSemaphoreCreateMutexStatic(&gKalmanFlagMutexBuffer);
        gRollAngleSetpointMutex = xSemaphoreCreateMutexStatic(&gRollAngleSetpointMutexBuffer);
        gRollAngularVelocityMutex = xSemaphoreCreateMutexStatic(&gRollAngularVelocityMutexBuffer);
    }

    void CreateQueues() {
        gI2C1BarometerQueue = xQueueCreateStatic(BAROMETERQUEUEDEPTH, sizeof(Common::BarometerData), gI2C1BarometerQueueStorage, &gI2C1BarometerQueueBuffer);
        gI2C1GPSQueue = xQueueCreateStatic(GPSQUEUEDEPTH, sizeof(Common::GPSData), gI2C1GPSQueueStorage, &gI2C1GPSQueueBuffer);
        gI2C1LidarQueue = xQueueCreateStatic(LIDARQUEUEDEPTH, sizeof(Common::LidarData), gI2C1LidarQueueStorage, &gI2C1LidarQueueBuffer);
        gI2C2IMUQueue = xQueueCreateStatic(IMUQUEUEDEPTH, sizeof(Common::IMUBatch), gI2C2IMUQueueStorage, &gI2C2IMUQueueBuffer);
        gI2C2MagQueue = xQueueCreateStatic(MAGQUEUEDEPTH, sizeof(Common::MagData), gI2C2MagQueueStorage, &gI2C2MagQueueBuffer);
        gI2C1VoltageQueue = xQueueCreateStatic(VOLTAGEQUEUEDEPTH, sizeof(Common::VoltageData), gI2C1VoltageQueueStorage, &gI2C1VoltageQueueBuffer);
        gCommandQueue = xQueueCreateStatic(COMMANDQUEUEDEPTH, Common::COMMAND_BUFFER_LENGTH, gCommandQueueStorage, &gCommandQueueBuffer);
        gSetTimeQueue = xQueueCreateStatic(SETTIMEQUEUEDEPTH, Common::TIME_STRING_LENGTH, gSetTimeQueueStorage, &gSetTimeQueueBuffer);
        gBarometerCalibrationQueue = xQueueCreateStatic(BAROMETERCALIBRATIONQUEUEDEPTH, sizeof(float), gBarometerCalibrationQueueStorage, &gBarometerCalibrationQueueBuffer);
    }

    void InitializeDefaults() {
        gFlightState = Common::FlightState::STANDBY;
        gSimEnabled = false;
        gCXFlag = false;
        gKalmanFlag = false;
        memset(gCommand, 0, sizeof(gCommand));
    }

    bool ResourcesReady() {
        return gSnapshotMutex != nullptr
            && gFlightStateMutex != nullptr
            && gSimEnabledMutex != nullptr
            && gStateVectorMutex != nullptr
            && gCommandMutex != nullptr
            && gCXFlagMutex != nullptr
            && gKalmanFlagMutex != nullptr
            && gRollAngleSetpointMutex != nullptr
            && gRollAngularVelocityMutex != nullptr
            && gI2C1BarometerQueue != nullptr
            && gI2C1GPSQueue != nullptr
            && gI2C1LidarQueue != nullptr
            && gI2C2IMUQueue != nullptr
            && gI2C2MagQueue != nullptr
            && gI2C1VoltageQueue != nullptr
            && gCommandQueue != nullptr
            && gSetTimeQueue != nullptr
            && gBarometerCalibrationQueue != nullptr;
    }
}

    void InitializeThreadSafeData() {
        CreateMutexes();
        CreateQueues();
        InitializeDefaults();

        if(!ResourcesReady()) {
            // Handle error: insufficient resources to create mutexes or queues
            Serial.println("Error initializing shared data mutexes or queues");
            while (true) {} // Halt execution
        }
    }
}
