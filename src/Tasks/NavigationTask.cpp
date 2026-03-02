#include "TaskHandler.h"




void Tasks::NavigationTask(void* pvParameters) {
    // Placeholder for navigation logic. This task would typically read the latest sensor snapshot,
    // run the navigation algorithm (e.g., an Extended Kalman Filter), and update the estimated state.
    // For now, it just runs an infinite loop with a delay.

    while (true) {
        // Example: Read the latest sensor snapshot.
        Common::SensorSnapshot snapshot;
        if (GetSnapshot(&snapshot)) {
            // Run navigation algorithm here using the snapshot data.
            // For example:
            // AdaptiveEKF::Update(snapshot);
        }

        // Delay to control task frequency (e.g., 50 Hz).
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}