
#include "TaskHandler.h"

namespace {
    constexpr uint16_t kSensorTaskStackDepth = 2048;
    constexpr UBaseType_t kSensorTaskPriority = 2;

    
}


namespace Tasks
{
    TaskHandle_t I2C1BusTaskHandle = nullptr;
    StaticTask_t I2C1BusTaskBuffer;

    TaskHandle_t I2C2BusTaskHandle = nullptr;
    StaticTask_t I2C2BusTaskBuffer;

    TaskHandle_t WriteTelemetryTaskHandle = nullptr;
    StaticTask_t WriteTelemetryTaskBuffer;
    TaskHandle_t DataLoggingTaskHandle = nullptr;
    StaticTask_t DataLoggingTaskBuffer;
    TaskHandle_t ReadCommandTaskHandle = nullptr;
    StaticTask_t ReadCommandTaskBuffer;

    TaskHandle_t ControlTaskHandle = nullptr;
    StaticTask_t ControlTaskBuffer;
    TaskHandle_t NavigationTaskHandle = nullptr;
    StaticTask_t NavigationTaskBuffer;

    TaskHandle_t MissionManagerTaskHandle = nullptr;
    TaskHandle_t WatchdogTaskHandle = nullptr;


}

void Tasks::InitializeTasks() {
    // Initialize all hardware once before tasks start reading.
    // const bool ok = gHardwareManager.Initialize();
    // if (!ok) {
    //     Serial.println("Hardware init failed");
    // }

    // Create the shared-data mutex before any tasks start.
    SharedData::InitializeThreadSafeData();

    // Sensor task owns sensor reads and updates the shared snapshot.
    // static StackType_t gSensorTaskStack[kSensorTaskStackDepth];
    // static StaticTask_t gSensorTaskBuffer;
    // xTaskCreateStatic(
    //     SensorTask, // entry function
    //     "SensorTask", // name
    //     kSensorTaskStackDepth, // stack size for tasks (words, not bytes)
    //     nullptr, // parameter passed to the task (not used here)
    //     kSensorTaskPriority, // priority (higher number means higher priority)
    //     gSensorTaskStack, &gSensorTaskBuffer); // task handle
}

bool Tasks::GetSnapshot(Common::SensorSnapshot* outSnapshot) {
    if (outSnapshot == nullptr || SharedData::gSnapshotMutex == nullptr) {
        return false;
    }

    if (xSemaphoreTake(SharedData::gSnapshotMutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    *outSnapshot = SharedData::gSnapshot;
    xSemaphoreGive(SharedData::gSnapshotMutex);
    return true;
}


