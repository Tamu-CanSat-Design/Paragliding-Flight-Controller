#include "HealthMonitor.h"

namespace {
// Human-readable labels for watchdog fault messages.
constexpr const char* kTaskNames[] = {
    "I2C1Bus",
    "I2C2Bus",
    "WriteTelemetry",
    "ReadCommand",
    "MissionManager",
};

TickType_t gLastKickTicks[static_cast<size_t>(HealthMonitor::TaskId::Count)] = {};
}

namespace HealthMonitor {

void Initialize() {
    const TickType_t now = xTaskGetTickCount();
    for (size_t i = 0; i < static_cast<size_t>(TaskId::Count); ++i) {
        gLastKickTicks[i] = now;
    }
}

void Kick(TaskId task) {
    gLastKickTicks[static_cast<size_t>(task)] = xTaskGetTickCount();
}

TickType_t LastKick(TaskId task) {
    return gLastKickTicks[static_cast<size_t>(task)];
}

const char* TaskName(TaskId task) {
    return kTaskNames[static_cast<size_t>(task)];
}

}
