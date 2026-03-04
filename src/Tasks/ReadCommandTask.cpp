#include "TaskHandler.h"

void Tasks::ReadCommandTask(void* pvParameters) {
    (void)pvParameters;
    char cmdBuf[100]; // Buffer for incoming command (adjust size as needed)
    


    while (true) {
        SharedData::commlink.ReadGroundRadio(cmdBuf, sizeof(cmdBuf));
        if (cmdBuf[0] != '\0') { // Check if we received a command
            SharedData::commlink.ParseCommand(cmdBuf);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
    }
}