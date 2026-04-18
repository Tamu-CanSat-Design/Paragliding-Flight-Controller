
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
    constexpr uint16_t kGuidanceTaskStackDepth = 2048;
    // constexpr uint16_t kWatchdogTaskStackDepth = 2048;



    constexpr UBaseType_t kI2C1BusTaskPriority = 1;
    constexpr UBaseType_t kI2C2BusTaskPriority = 2;
    constexpr UBaseType_t kWriteTelemetryTaskPriority = 3;
    constexpr UBaseType_t kDataLoggingTaskPriority = 4;
    constexpr UBaseType_t kReadCommandTaskPriority = 5;
    constexpr UBaseType_t kControlTaskPriority = 6;
    constexpr UBaseType_t kNavigationTaskPriority = 7;
    constexpr UBaseType_t kMissionManagerTaskPriority = 8;
    constexpr UBaseType_t kGuidanceTaskPriority = 9;


    
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

    TaskHandle_t GuidanceTaskHandle = nullptr;
    StaticTask_t GuidanceTaskBuffer;
    StackType_t GuidanceTaskStack[kGuidanceTaskStackDepth];

    TaskHandle_t NavigationTaskHandle = nullptr;
    StaticTask_t NavigationTaskBuffer;
    StackType_t NavigationTaskStack[kNavigationTaskStackDepth];


    TaskHandle_t MissionManagerTaskHandle = nullptr;
    StaticTask_t MissionManagerTaskBuffer;
    StackType_t MissionManagerTaskStack[kMissionManagerTaskStackDepth];

    TaskHandle_t WatchdogTaskHandle = nullptr;



    void InitializeTasks() {
        

        // Create the shared-data before any tasks start.
        SharedData::InitializeThreadSafeData();

        // Create Each Task
        I2C1BusTaskHandle = xTaskCreateStatic(
            I2C1BusTask, // entry function
            "I2C Task 1", // name
            kI2C1BusTaskStackDepth, // stack size for tasks (words, not bytes)
            nullptr, // parameter passed to the task (not used here)
            kI2C1BusTaskPriority, // priority (higher number means higher priority)
            I2C1BusTaskStack, &I2C1BusTaskBuffer); // task handle

        I2C2BusTaskHandle = xTaskCreateStatic(
            I2C2BusTask, 
            "I2C Task 2", 
            kI2C2BusTaskStackDepth, 
            nullptr, 
            kI2C2BusTaskPriority, 
            I2C2BusTaskStack, &I2C2BusTaskBuffer);

        
        WriteTelemetryTaskHandle = xTaskCreateStatic(
            WriteTelemetryTask, 
            "Write Telemetry", 
            kWriteTelemetryTaskStackDepth, 
            nullptr, 
            kWriteTelemetryTaskPriority, 
            WriteTelemetryTaskStack, &WriteTelemetryTaskBuffer);
        
        DataLoggingTaskHandle = xTaskCreateStatic(
            DataLoggingTask,
            "Data Logging",
            kDataLoggingTaskStackDepth,
            nullptr,
            kDataLoggingTaskPriority, 
            DataLoggingTaskStack, &DataLoggingTaskBuffer);
        
        ReadCommandTaskHandle = xTaskCreateStatic(
            ReadCommandTask,
            "Read Command",
            kReadCommandTaskStackDepth,
            nullptr,
            kReadCommandTaskPriority,
            ReadCommandTaskStack, &ReadCommandTaskBuffer);

        ControlTaskHandle = xTaskCreateStatic(
            ControlTask,
            "Control Task",
            kControlTaskStackDepth,
            nullptr,
            kControlTaskPriority, 
            ControlTaskStack, &ControlTaskBuffer);

        NavigationTaskHandle = xTaskCreateStatic(
            NavigationTask,
            "Navigation Task",
            kNavigationTaskStackDepth,
            nullptr,
            kNavigationTaskPriority, 
            NavigationTaskStack, &NavigationTaskBuffer);
        
        GuidanceTaskHandle = xTaskCreateStatic(
            GuidanceTask,
            "Guidance Task",
            kGuidanceTaskStackDepth,
            nullptr,
            kGuidanceTaskPriority, 
            GuidanceTaskStack, &GuidanceTaskBuffer);

        MissionManagerTaskHandle = xTaskCreateStatic(
            MissionManagerTask,
            "Mission Manager Task",
            kMissionManagerTaskStackDepth,
            nullptr,
            kMissionManagerTaskPriority,
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
