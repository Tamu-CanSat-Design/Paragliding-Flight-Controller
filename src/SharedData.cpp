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


    Common::FlightState gSD_State;
    SemaphoreHandle_t gSD_StateMutex = nullptr;
    static StaticSemaphore_t gSD_StateMutexBuffer;



    Common::StateVector gStateVector;
    SemaphoreHandle_t gStateVectorMutex = nullptr;
    static StaticSemaphore_t gStateVectorMutexBuffer;



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
        gSD_StateMutex = xSemaphoreCreateMutexStatic(&gSD_StateMutexBuffer);
        gStateVectorMutex = xSemaphoreCreateMutexStatic(&gStateVectorMutexBuffer);
    }
}
