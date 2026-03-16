#include "TaskHandler.h"

#include <cstring>

void Tasks::ReadCommandTask(void* pvParameters) {
    (void)pvParameters;
    char cmdBuf[32] = {0}; // Buffer for incoming command (adjust size as needed)
    


    while (true) {
        HealthMonitor::Kick(HealthMonitor::TaskId::ReadCommand);

        SharedData::commlink.ReadGroundRadio(cmdBuf, sizeof(cmdBuf));
        if (cmdBuf[0] != '\0') { // Check if we received a command
            if (SharedData::commlink.CheckCommand(cmdBuf)) {
                xQueueSend(SharedData::gCommandQueue, &cmdBuf, portMAX_DELAY); // Send command to MissionManager
            }
            // Clear the local buffer after handling one command so we do not
            // re-enqueue the same payload on every polling iteration.
            memset(cmdBuf, 0, sizeof(cmdBuf));
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
    }
}
