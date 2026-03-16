#ifndef __TASKS_H__
#define __TASKS_H__
#include <Arduino.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include "SharedData.h"
#include "HealthMonitor.h"

#include "Hardware/Hardware.h"
#include "GNC/Controller.h"
#include "Common.h"

namespace Tasks
{
    void InitializeTasks();
    bool GetSnapshot(Common::SensorSnapshot* outSnapshot);

    //Always running
    void I2C1BusTask(void* pvParameters); // GPS, Barometer, Lidar
    void I2C2BusTask(void* pvParameters); // IMU, Magnetometer, Voltage

    //Always running
    void WriteTelemetryTask(void* pvParameters);
    void DataLoggingTask(void* pvParameters);
    void ReadCommandTask(void* pvParameters);

    //Start on payload release
    void ControlTask(void* pvParameters);
    void GuidanceTask(void* pvParameters);
    //Start on Ascent
    void NavigationTask(void* pvParameters);
    
    //Always running
    void MissionManagerTask(void* pvParameters);
    void WatchdogTask(void* pvParameters);
    
    

    extern TaskHandle_t I2C1BusTaskHandle;
    extern TaskHandle_t I2C2BusTaskHandle;

    extern TaskHandle_t WriteTelemetryTaskHandle;
    extern TaskHandle_t DataLoggingTaskHandle;
    extern TaskHandle_t ReadCommandTaskHandle;

    extern TaskHandle_t ControlTaskHandle;
    extern TaskHandle_t NavigationTaskHandle;
    extern TaskHandle_t GuidanceTaskHandle;

    extern TaskHandle_t MissionManagerTaskHandle;
    extern TaskHandle_t WatchdogTaskHandle;
    

    

};









#endif //__TASKS_H__
