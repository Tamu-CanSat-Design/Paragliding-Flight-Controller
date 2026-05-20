#include "TaskHandler.h"
#include "Hardware/Hardware.h"
#include "Common.h"

namespace{
    char setTimeCommandBuffer[Common::COMMAND_BUFFER_LENGTH] = {};
}

void Tasks::RTCTask(void *pvParameters) {


    while(true) {

        // Check if there is a command to set the RTC time
        while(xQueueReceive(SharedData::gSetTimeQueue, setTimeCommandBuffer, 0) == pdTRUE) {
            // Parse the command to extract the time value (assuming it's a H:M:S timestamp)
            int hours, minutes, seconds;
            if (sscanf(setTimeCommandBuffer, "%d:%d:%d", &hours, &minutes, &seconds) == 3) {
                // Convert the parsed time to a time_t value (assuming the date is not important and we just want to set the time)
                //Date is always 6/6/2026
                time_t currentTime = time(nullptr);
                struct tm timeInfo = *localtime(&currentTime);
                timeInfo.tm_hour = hours;
                timeInfo.tm_min = minutes;
                timeInfo.tm_sec = seconds;
                timeInfo.tm_mday = 6;
                timeInfo.tm_mon = 5; // June (0-indexed)
                timeInfo.tm_year = 126; // 2026 - 1900
                time_t newTime = mktime(&timeInfo);

                // Set the RTC time using the RTC driver
                
                Hardware::rtc_.SetTime(newTime);
            }


            if(xSemaphoreTake(SharedData::gSnapshotMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                SharedData::gSnapshot.timeData = Hardware::rtc_.GetTime();
                xSemaphoreGive(SharedData::gSnapshotMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
    }
}