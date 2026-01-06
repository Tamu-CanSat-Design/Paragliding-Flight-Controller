#ifndef __TASKS_H__
#define __TASKS_H__


namespace Tasks
{
    void InitializeTasks();

    void SensorTask(void* pvParameters);
    void GPSTask(void* pvParameters);
    void WriteTelemetryTask(void* pvParameters);
    void DataLoggingTask(void* pvParameters);
    void ControlTask(void* pvParameters);
    void MissionManagerTask(void* pvParameters);
    void WatchdogTask(void* pvParameters);
    void ReadCommandTask(void* pvParameters);
    
};









#endif //__TASKS_H__