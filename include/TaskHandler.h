#ifndef __TASKS_H__
#define __TASKS_H__
#include <Arduino.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include "GNC/AdaptiveEKF.h"
#include "Common.h"

namespace Tasks
{
    void InitializeTasks();
    bool GetSnapshot(Common::SensorSnapshot* outSnapshot);

    void I2C1BusTask(void* pvParameters); // GPS, Barometer, Lidar
    void I2C2BusTask(void* pvParameters); // IMU, Magnetometer, Voltage

    void WriteTelemetryTask(void* pvParameters);
    void DataLoggingTask(void* pvParameters);
    void ControlTask(void* pvParameters);
    void NavigationTask(void* pvParameters);
    void MissionManagerTask(void* pvParameters);
    void WatchdogTask(void* pvParameters);
    void ReadCommandTask(void* pvParameters);
    
};









#endif //__TASKS_H__
