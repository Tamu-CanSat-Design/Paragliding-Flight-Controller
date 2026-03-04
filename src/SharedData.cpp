#include "SharedData.h"

namespace{
    UBaseType_t BAROMETERQUEUEDEPTH = 10;
    UBaseType_t GPSQUEUEDEPTH = 10;
    UBaseType_t LIDARQUEUEDEPTH = 10;
    UBaseType_t IMUQUEUEDEPTH = 10;
    UBaseType_t MAGQUEUEDEPTH = 10;
    UBaseType_t VOLTAGEQUEUEDEPTH = 10;

}


namespace SharedData {
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

    Common::StateVector gStateVector;
    SemaphoreHandle_t gStateVectorMutex = nullptr;
    static StaticSemaphore_t gStateVectorMutexBuffer;

    char gCommand[10] = {0}; // Initialize with null terminator
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
    static StaticQueue_t gI2C2VoltageQueueBuffer;
    QueueHandle_t gI2C2IMUQueue = nullptr;
    QueueHandle_t gI2C2MagQueue = nullptr;
    QueueHandle_t gI2C2VoltageQueue = nullptr;



    



    void InitializeThreadSafeData() {
        gSnapshotMutex = xSemaphoreCreateMutexStatic(&gSnapshotMutexBuffer);
        gFlightStateMutex = xSemaphoreCreateMutexStatic(&gFlightStateMutexBuffer);
        gSimEnabledMutex = xSemaphoreCreateMutexStatic(&gSimEnabledMutexBuffer);
        gStateVectorMutex = xSemaphoreCreateMutexStatic(&gStateVectorMutexBuffer);
        gCommandMutex = xSemaphoreCreateMutexStatic(&gCommandMutexBuffer);
    }
}
