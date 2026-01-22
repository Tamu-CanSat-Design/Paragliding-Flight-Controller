#include <Arduino.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include "HardwareManager.h"
#include "Tasks/Tasks.h"

namespace {
    // Sensor cadence used for basic attitude/heading updates.
    constexpr TickType_t kSensorTaskPeriodTicks = pdMS_TO_TICKS(10);  // 100 Hz.
    constexpr uint16_t kSensorTaskStackDepth = 2048;
    constexpr UBaseType_t kSensorTaskPriority = 2;

    HardwareManager gHardwareManager;
    SensorSnapshot gSnapshot;
    SemaphoreHandle_t gSnapshotMutex = nullptr;

    void UpdateSnapshotLocked() {
        // Copy the manager's cached snapshot into the shared buffer.
        gSnapshot = gHardwareManager.GetSnapshot();
    }
}  // namespace

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
        SensorTask,
        "SensorTask",
        kSensorTaskStackDepth,
        nullptr,
        kSensorTaskPriority,
        nullptr);
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

void Tasks::SensorTask(void* pvParameters) {
    (void)pvParameters;

    // vTaskDelayUntil keeps a stable period across iterations.
    TickType_t lastWakeTime = xTaskGetTickCount();
    for (;;) {
        gHardwareManager.ReadSensors();

        if (gSnapshotMutex != nullptr && xSemaphoreTake(gSnapshotMutex, portMAX_DELAY) == pdTRUE) {
            UpdateSnapshotLocked();
            xSemaphoreGive(gSnapshotMutex);
        }

        vTaskDelayUntil(&lastWakeTime, kSensorTaskPeriodTicks);
    }
}
