#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include <FreeRTOS.h>
#include <task.h>

#include <cstdint>

// WORK IN PROGRESS:
// This heartbeat tracker is part of an unfinished watchdog system. It currently
// supports task liveness timestamps for bring-up and basic fault detection, but
// it is not yet a complete flight-ready fault-management interface.

namespace HealthMonitor {

enum class TaskId : uint8_t {
    I2C1Bus = 0,
    I2C2Bus,
    WriteTelemetry,
    ReadCommand,
    MissionManager,
    Count,
};

void Initialize();
void Kick(TaskId task);
TickType_t LastKick(TaskId task);
const char* TaskName(TaskId task);

}

#endif // HEALTH_MONITOR_H
