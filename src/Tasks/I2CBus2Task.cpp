// #include "Tasks.h"
// #include "SharedData.h"

// namespace {
//     // Sensor cadence used for basic attitude/heading updates.
//     // "k" means "constant" in our naming convention, and "Ticks" means this is a duration in FreeRTOS ticks.
//     constexpr TickType_t kSensorTaskPeriodTicks = pdMS_TO_TICKS(10);  // 100 Hz.

//     void UpdateSnapshotLocked() {
//         // Copy the manager's cached snapshot into the shared buffer.
//         // SharedData::gSnapshot = gHardwareManager.GetSnapshot();
//     }
// }


// /**
//  * SensorTask is responsible for reading all sensors at a fixed cadence and updating the shared snapshot.
//  * It wakes up, reads sensors, publishes the latest snapshot, and then sleeps until the next period.
//  */
// void Tasks::SensorTask(void* pvParameters) {
//     (void)pvParameters; // silencing warning about unused parameter

//     // vTaskDelayUntil keeps a stable period across iterations.
//     TickType_t lastWakeTime = xTaskGetTickCount(); // basically "now" in ticks
//     for (;;) { // infinite loop, tasks never return in FreeRTOS
//         // gHardwareManager.ReadSensors(); // actually retrieve the data from the sensors (this actually modifies the snapshot)

//         // if mutex exists, try to take it and update the shared snapshot with the latest data from the hardware manager
//         if (SharedData::gSnapshotMutex != nullptr && xSemaphoreTake(SharedData::gSnapshotMutex, portMAX_DELAY) == pdTRUE) {
//             UpdateSnapshotLocked();
//             // release the mutex so other tasks can read the updated snapshot
//             xSemaphoreGive(SharedData::gSnapshotMutex);
//         }


//         // periodic timing mechanism
//         vTaskDelayUntil(&lastWakeTime, kSensorTaskPeriodTicks);
//     }
// }