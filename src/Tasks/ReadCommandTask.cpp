#include "TaskHandler.h"

void Tasks::ReadCommandTask(void* pvParameters) {
    (void)pvParameters;
    char cmdBuf[32] = {0}; // Buffer for incoming command (adjust size as needed)
    


    while (true) {
        SharedData::commlink.ReadGroundRadio(cmdBuf, sizeof(cmdBuf));
        if (cmdBuf[0] != '\0') { // Check if we received a command
            if (SharedData::commlink.CheckCommand(cmdBuf)) {
                xQueueSend(SharedData::gCommandQueue, &cmdBuf, portMAX_DELAY); // Send command to MissionManager
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
    }
}