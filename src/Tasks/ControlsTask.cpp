#include "TaskHandler.h"
#include "GNC/Controller.h"
#include "GNC/ControllerMixer.h"

namespace{
    float rollAngleSetpoint = 0.0f;
    
}


void Tasks::ControlTask(void* pvParameters) {
    pvParameters = pvParameters; // Avoid unused parameter warning
    // Placeholder for control logic. This task would typically read the latest sensor snapshot,
    // run the control algorithms, and send commands to the actuators.
    // For now, it just runs an infinite loop with a delay.

    while (true) {
        // Example: Read the latest sensor snapshot.
        Common::SensorSnapshot snapshot;
        if (GetSnapshot(&snapshot)) {
            // Run control algorithm here using the snapshot data.
            // For example:
            // ControllerMixer::Update(snapshot);
        }



        // Delay to control task frequency (e.g., 50 Hz).
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}