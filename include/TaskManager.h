#ifndef __TASKMANAGER_H__
#define __TASKMANAGER_H__

#include "Common.h"
// Define all tasks here




namespace Tasks
{
    void InitializeTasks();
    bool GetSnapshot(Common::SensorSnapshot* outSnapshot);

    void SensorTask(void* pvParameters);
    void GPSTask(void* pvParameters);
    void WriteTelemetryTask(void* pvParameters);
    void DataLoggingTask(void* pvParameters);
    void ControlTask(void* pvParameters);
    void MissionManagerTask(void* pvParameters);
    void WatchdogTask(void* pvParameters);
    void ReadCommandTask(void* pvParameters);
    
};


#endif // __TASKMANAGER_H__