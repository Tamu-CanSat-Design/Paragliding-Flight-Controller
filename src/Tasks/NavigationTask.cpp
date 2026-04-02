#include "TaskHandler.h"
#include "GNC/AdaptiveEKF.h"
namespace {
    // This file implements the NavigationTask, which runs the EKF and state estimation logic.
    // It receives sensor data from the I2C bus tasks via queues, updates the EKF, and writes
    // the estimated state to shared memory for use by other tasks (e.g., control, telemetry).

    AdaptiveEKF ekf; // EKF instance (could also be a pointer if dynamic allocation is needed)
    Common::IMUData       latestImu     = {}; //May put this within the EKF class
    Common::MagData       latestMag     = {};
    Common::BarometerData latestBaro    = {};
    Common::GPSData       latestGps     = {};
    Common::LidarData     latestLidar   = {};
    Common::VoltageData   latestVoltage = {};
}

void Tasks::NavigationTask(void* pvParameters) {
    pvParameters = pvParameters; // Avoid unused parameter warning
    //Predict


    //Pseudocode for navigation task:
    while (true) {

        // Suspend Task forever if simulation mode 
        if(ulTaskNotifyTake(pdFALSE,0) % 2 == 1){
            vTaskSuspend(NULL);
        }


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
        while(xQueueReceive(SharedData::gI2C2VoltageQueue, &latestVoltage, 0) == pdTRUE) {
            // Voltage is not fed into the EKF; draining prevents queue overflow.
        }
    //EKF Predict
    //unlock flight state mutex and read current flight state for adaptive noise tuning
        ekf.Predict();
    
    //EKF Update when each queue notification is received
    //Write output to the shared state vector with a timestamp
    //vTaskDelay for a short period to yield to other tasks (e.g.,
    //control task might want to run at a different frequency)
        vTaskDelay(pdMS_TO_TICKS(2)); // Adjust delay as needed for desired update rate
    }
}
