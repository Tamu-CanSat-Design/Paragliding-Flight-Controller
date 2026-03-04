#include "TaskHandler.h"



void Tasks::NavigationTask(void* pvParameters) {
    //Predict
    static AdaptiveEKF ekf;
    ekf.Initialize();

    static Common::IMUData       latestImu   = {}; //May put this within the EKF class
    static Common::MagData      latestMag   = {};
    static Common::BarometerData latestBaro  = {};
    static Common::GPSData       latestGps   = {};
    static Common::LidarData     latestLidar = {};


    //Pseudocode for navigation task:
    while (true) {
        if(xSemaphoreTake(SharedData::gFlightStateMutex, 0) == pdTRUE) {
            ekf.SetFlightState(SharedData::gFlightState);
            xSemaphoreGive(SharedData::gFlightStateMutex);
        }
        else{} 

        while(xQueueReceive(SharedData::gI2C1BarometerQueue, &latestBaro, 0) == pdTRUE) {
            ekf.UpdateBarometer(latestBaro);
        }
        while(xQueueReceive(SharedData::gI2C1GPSQueue, &latestGps, 0) == pdTRUE) {
            ekf.UpdateGPS(latestGps);
        }
        while(xQueueReceive(SharedData::gI2C1LidarQueue, &latestLidar, 0) == pdTRUE) {
            ekf.UpdateLidar(latestLidar);
        }
        while(xQueueReceive(SharedData::gI2C2IMUQueue, &latestImu, 0) == pdTRUE) {
            ekf.UpdateIMU(latestImu);
        }
        while(xQueueReceive(SharedData::gI2C2MagQueue, &latestMag, 0) == pdTRUE) {
            ekf.UpdateMagnetometer(latestMag);
        }
    //EKF Predict
    //unlock flight state mutex and read current flight state for adaptive noise tuning
        ekf.Predict();
    
    //EKF Update when each queue notification is received
    //Write output to the shared state vector with a timestamp
    //vTaskDelay for a short period to yield to other tasks (e.g.,
    }
    return;

}