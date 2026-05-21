#include "TaskHandler.h"

void Tasks::ReadCommandTask(void* pvParameters) {
    (void)pvParameters;
    char cmdBuf[32] = {0}; // Buffer for incoming command (adjust size as needed)
    


    while (true) {
        Hardware::commlink_.ReadGroundRadio(cmdBuf, sizeof(cmdBuf));
        if (cmdBuf[0] != '\0') { // Check if we received a command
            if (Hardware::commlink_.CheckCommand(cmdBuf)) {
                xQueueSend(SharedData::gCommandQueue, cmdBuf, 0); // Send command to MissionManager if queue has room.
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Adjust delay as needed
    }
}
