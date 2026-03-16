#include "TaskHandler.h"

#include <cstdio>

// WORK IN PROGRESS:
// This watchdog currently implements a basic heartbeat-and-stack monitor with
// hard MCU reset on detected failure. It is useful for bring-up, but it is not
// yet a complete flight-ready fault-management strategy. In particular, it does
// not yet distinguish recoverable faults from unrecoverable faults, persist the
// reset reason across reboot, or implement a post-reset safe-mode policy.

namespace {
// Time constants are stored in scheduler ticks because FreeRTOS APIs operate on
// ticks rather than milliseconds.
constexpr TickType_t kWatchdogPollPeriod = pdMS_TO_TICKS(500);
constexpr TickType_t kStartupGracePeriod = pdMS_TO_TICKS(5000);

// Minimum acceptable free-stack watermark, measured in stack words.
constexpr UBaseType_t kMinimumStackWatermarkWords = 128;

// Static configuration for one monitored task. The handle is stored as a
// pointer-to-handle so the watchdog sees the real task handle after creation.
struct WatchedTaskConfig {
    HealthMonitor::TaskId id;
    TaskHandle_t* handle;
    TickType_t timeoutTicks;
};

// Core always-running tasks that must keep making forward progress.
constexpr WatchedTaskConfig kWatchedTasks[] = {
    {HealthMonitor::TaskId::I2C1Bus, &Tasks::I2C1BusTaskHandle, pdMS_TO_TICKS(250)},
    {HealthMonitor::TaskId::I2C2Bus, &Tasks::I2C2BusTaskHandle, pdMS_TO_TICKS(250)},
    {HealthMonitor::TaskId::WriteTelemetry, &Tasks::WriteTelemetryTaskHandle, pdMS_TO_TICKS(2500)},
    {HealthMonitor::TaskId::ReadCommand, &Tasks::ReadCommandTaskHandle, pdMS_TO_TICKS(500)},
    {HealthMonitor::TaskId::MissionManager, &Tasks::MissionManagerTaskHandle, pdMS_TO_TICKS(500)},
};

// Reset is terminal for this code path, so this helper never returns.
[[noreturn]] void ResetMcu(const char* reason) {
    Serial.printf("Watchdog reset: %s\n", reason);
    Serial.flush();
    delay(50);
    SCB_AIRCR = 0x05FA0004;
    while (true) {
    }
}
}

void Tasks::WatchdogTask(void* pvParameters) {
    (void)pvParameters;

    // Start all heartbeat timestamps at the current tick to avoid false faults
    // during boot.
    HealthMonitor::Initialize();
    const TickType_t startTick = xTaskGetTickCount();

    while (true) {
        const TickType_t now = xTaskGetTickCount();

        // Give tasks time to finish startup before enforcing timeouts.
        if ((now - startTick) >= kStartupGracePeriod) {
            for (const auto& watchedTask : kWatchedTasks) {
                // Skip tasks that were never created.
                if (watchedTask.handle == nullptr || *watchedTask.handle == nullptr) {
                    continue;
                }

                // A task that stops updating its heartbeat is treated as stalled.
                const TickType_t elapsed = now - HealthMonitor::LastKick(watchedTask.id);
                if (elapsed > watchedTask.timeoutTicks) {
                    char reason[96];
                    std::snprintf(
                        reason,
                        sizeof(reason),
                        "%s stalled for %lu ms",
                        HealthMonitor::TaskName(watchedTask.id),
                        static_cast<unsigned long>(pdTICKS_TO_MS(elapsed)));
                    ResetMcu(reason);
                }

                // Low remaining stack is also treated as fatal to avoid silent
                // overflow and memory corruption.
                const UBaseType_t stackWatermark = uxTaskGetStackHighWaterMark(*watchedTask.handle);
                if (stackWatermark <= kMinimumStackWatermarkWords) {
                    char reason[96];
                    std::snprintf(
                        reason,
                        sizeof(reason),
                        "%s low stack watermark %u words",
                        HealthMonitor::TaskName(watchedTask.id),
                        static_cast<unsigned int>(stackWatermark));
                    ResetMcu(reason);
                }
            }
        }

        vTaskDelay(kWatchdogPollPeriod);
    }
}
