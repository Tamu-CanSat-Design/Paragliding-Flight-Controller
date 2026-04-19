#include "TaskHandler.h"
#include "GNC/Controller.h"

namespace{
    float rollAngleSetpoint = 0.0f;
    float leftAngleCmd = 0.0f;
    float rightAngleCmd = 0.0f;
    ControllerMixer controller;
    Common::StateVector latestStateVector;
    float dt = 20.0f;
}


void Tasks::ControlTask(void* pvParameters) {
    pvParameters = pvParameters; // Avoid unused parameter warning
    // Placeholder for control logic. This task would typically read the latest sensor snapshot,
    // run the control algorithms, and send commands to the actuators.
    // For now, it just runs an infinite loop with a delay.

    while (true) {
        // Example: Read the latest sensor snapshot.
        // This is a placeholder - replace with actual sensor reading logic.
        if(xSemaphoreTake(SharedData::gStateVectorMutex, 0) == pdTRUE) {
            latestStateVector = SharedData::gStateVector;
            xSemaphoreGive(SharedData::gStateVectorMutex);
        }
        else{}

         // Get the latest roll angle setpoint from the guidance task   
        if(xSemaphoreTake(SharedData::gRollAngleSetpointMutex, 0) == pdTRUE) {
            rollAngleSetpoint = SharedData::gRollAngleSetpoint;
            xSemaphoreGive(SharedData::gRollAngleSetpointMutex);
        }
        else{}

        // Update controller gains based on current state
        controller.UpdateGains(latestStateVector);
        controller.CalculateCommands(latestStateVector, rollAngleSetpoint, dt/1000.0f); // Convert ms to seconds

        leftAngleCmd = controller.GetLeftOutput();
        rightAngleCmd = controller.GetRightOutput();

        //Send commands to servos

        // Delay to control task frequency (e.g., 50 Hz).
        vTaskDelay(pdMS_TO_TICKS(dt));
    }
}