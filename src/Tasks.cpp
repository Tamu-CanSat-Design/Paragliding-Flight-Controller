#include <Arduino.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include "HardwareManager.h"
#include "Tasks/Tasks.h"

namespace {
    // Sensor cadence used for basic attitude/heading updates.
    // "k" means "constant" in our naming convention, and "Ticks" means this is a duration in FreeRTOS ticks.
    constexpr TickType_t kSensorTaskPeriodTicks = pdMS_TO_TICKS(10);  // 100 Hz.
    constexpr uint16_t kSensorTaskStackDepth = 2048;
    constexpr UBaseType_t kSensorTaskPriority = 2;

    // "g" means "global" in our naming convention. These are shared across tasks in this file.
    HardwareManager gHardwareManager;
    SensorSnapshot gSnapshot;
    SemaphoreHandle_t gSnapshotMutex = nullptr;

    void UpdateSnapshotLocked() {
        // Copy the manager's cached snapshot into the shared buffer.
        gSnapshot = gHardwareManager.GetSnapshot();
    }
} 

void Tasks::InitializeTasks() {
    // Initialize all hardware once before tasks start reading.
    const bool ok = gHardwareManager.Initialize();
    if (!ok) {
        Serial.println("Hardware init failed");
    }

    // Snapshot mutex protects readers from seeing partially-updated data.
    gSnapshotMutex = xSemaphoreCreateMutex();
    if (gSnapshotMutex != nullptr) {
        if (xSemaphoreTake(gSnapshotMutex, portMAX_DELAY) == pdTRUE) {
            UpdateSnapshotLocked();
            xSemaphoreGive(gSnapshotMutex);
        }
    }

    // Sensor task owns sensor reads and updates the shared snapshot.
    xTaskCreate(
        SensorTask, // entry function
        "SensorTask", // name
        kSensorTaskStackDepth, // stack size for tasks (words, not bytes)
        nullptr, // parameter passed to the task (not used here)
        kSensorTaskPriority, // priority (higher number means higher priority)
        nullptr); // task handle (not used here, so we pass)
}

bool Tasks::GetSnapshot(SensorSnapshot* outSnapshot) {
    if (outSnapshot == nullptr || gSnapshotMutex == nullptr) {
        return false;
    }

    if (xSemaphoreTake(gSnapshotMutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    *outSnapshot = gSnapshot;
    xSemaphoreGive(gSnapshotMutex);
    return true;
}

/**
 * SensorTask is responsible for reading all sensors at a fixed cadence and updating the shared snapshot.
 * It wakes up, reads sensors, publishes the latest snapshot, and then sleeps until the next period.
 */
void Tasks::SensorTask(void* pvParameters) {
    (void)pvParameters; // silencing warning about unused parameter

    // vTaskDelayUntil keeps a stable period across iterations.
    TickType_t lastWakeTime = xTaskGetTickCount(); // basically "now" in ticks
    for (;;) { // infinite loop, tasks never return in FreeRTOS
        gHardwareManager.ReadSensors(); // actually retrieve the data from the sensors (this actually modifies the snapshot)

        // if mutex exists, try to take it and update the shared snapshot with the latest data from the hardware manager
        if (gSnapshotMutex != nullptr && xSemaphoreTake(gSnapshotMutex, portMAX_DELAY) == pdTRUE) {
            UpdateSnapshotLocked();
            // release the mutex so other tasks can read the updated snapshot
            xSemaphoreGive(gSnapshotMutex);
        }


        // periodic timing mechanism
        vTaskDelayUntil(&lastWakeTime, kSensorTaskPeriodTicks);
    }
}
