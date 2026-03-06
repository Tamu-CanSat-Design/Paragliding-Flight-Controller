
#include "TaskHandler.h"

namespace {
    constexpr uint16_t kI2C1BusTaskStackDepth = 2048;
    constexpr uint16_t kI2C2BusTaskStackDepth = 2048;
    constexpr uint16_t kWriteTelemetryTaskStackDepth = 2048;
    constexpr uint16_t kDataLoggingTaskStackDepth = 2048;
    constexpr uint16_t kReadCommandTaskStackDepth = 2048;
    constexpr uint16_t kControlTaskStackDepth = 2048;
    constexpr uint16_t kNavigationTaskStackDepth = 2048;
    constexpr uint16_t kMissionManagerTaskStackDepth = 2048;




    constexpr UBaseType_t kI2C1BusTaskPriority = 2;
    

    
}


namespace Tasks
{
    TaskHandle_t I2C1BusTaskHandle = nullptr;
    StaticTask_t I2C1BusTaskBuffer;
    StackType_t I2C1BusTaskStack[kI2C1BusTaskStackDepth];

    TaskHandle_t I2C2BusTaskHandle = nullptr;
    StaticTask_t I2C2BusTaskBuffer;
    StackType_t I2C2BusTaskStack[kI2C2BusTaskStackDepth];

    TaskHandle_t WriteTelemetryTaskHandle = nullptr;
    StaticTask_t WriteTelemetryTaskBuffer;
    StackType_t WriteTelemetryTaskStack[kWriteTelemetryTaskStackDepth];

    TaskHandle_t DataLoggingTaskHandle = nullptr;
    StaticTask_t DataLoggingTaskBuffer;
    StackType_t DataLoggingTaskStack[kDataLoggingTaskStackDepth];

    TaskHandle_t ReadCommandTaskHandle = nullptr;
    StaticTask_t ReadCommandTaskBuffer;
    StackType_t ReadCommandTaskStack[kReadCommandTaskStackDepth];

    TaskHandle_t ControlTaskHandle = nullptr;
    StaticTask_t ControlTaskBuffer;
    StackType_t ControlTaskStack[kControlTaskStackDepth];

    TaskHandle_t NavigationTaskHandle = nullptr;
    StaticTask_t NavigationTaskBuffer;
    StackType_t NavigationTaskStack[kNavigationTaskStackDepth];


    TaskHandle_t MissionManagerTaskHandle = nullptr;
    TaskHandle_t WatchdogTaskHandle = nullptr;
    StaticTask_t MissionManagerTaskBuffer;
    StackType_t MissionManagerTaskStack[kMissionManagerTaskStackDepth];



    void Tasks::InitializeTasks() {
        // Initialize all hardware once before tasks start reading.
        // const bool ok = gHardwareManager.Initialize();
        // if (!ok) {
        //     Serial.println("Hardware init failed");
        // }

        // Create the shared-data mutex before any tasks start.
        SharedData::InitializeThreadSafeData();

        // Create Each Task

        xTaskCreateStatic(
            I2C1BusTask, // entry function
            "I2C Task 1", // name
            kI2C1BusTaskStackDepth, // stack size for tasks (words, not bytes)
            nullptr, // parameter passed to the task (not used here)
            kI2C1BusTaskPriority, // priority (higher number means higher priority)
            I2C1BusTaskStack, &I2C1BusTaskBuffer); // task handle

        xTaskCreateStatic(
            I2C2BusTask, 
            "I2C Task 2", 
            kI2C2BusTaskStackDepth, 
            nullptr, 
            2, //update
            I2C2BusTaskStack, &I2C2BusTaskBuffer);

        xTaskCreateStatic(
            WriteTelemetryTask, 
            "Write Telemetry", 
            kWriteTelemetryTaskStackDepth, 
            nullptr, 
            1, //update
            WriteTelemetryTaskStack, &WriteTelemetryTaskBuffer);
        
        xTaskCreateStatic(
            DataLoggingTask,
            "Data Logging",
            kDataLoggingTaskStackDepth,
            nullptr,
            1, //update
            DataLoggingTaskStack, &DataLoggingTaskBuffer);
        
        xTaskCreateStatic(
            ReadCommandTask,
            "Read Command",
            kReadCommandTaskStackDepth,
            nullptr,
            1, //update
            ReadCommandTaskStack, &ReadCommandTaskBuffer);

        xTaskCreateStatic(
            ControlTask,
            "Control Task",
            kControlTaskStackDepth,
            nullptr,
            1, //update
            ControlTaskStack, &ControlTaskBuffer);

        xTaskCreateStatic(
            NavigationTask,
            "Navigation Task",
            kNavigationTaskStackDepth,
            nullptr,
            1, //update
            NavigationTaskStack, &NavigationTaskBuffer);
        
        xTaskCreateStatic(
            MissionManagerTask,
            "Mission Manager Task",
            kMissionManagerTaskStackDepth,
            nullptr,
            1, //update
            MissionManagerTaskStack, &MissionManagerTaskBuffer);

        
    }


    bool GetSnapshot(Common::SensorSnapshot* outSnapshot) {
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


}