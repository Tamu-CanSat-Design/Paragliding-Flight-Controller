#include "TaskHandler.h"
#include "GNC/AdaptiveEKF.h"
namespace {
    // This file implements the NavigationTask, which runs the EKF and state estimation logic.
    // It receives sensor data from the I2C bus tasks via queues, updates the EKF, and writes
    // the estimated state to shared memory for use by other tasks (e.g., control, telemetry).
    //
    // IMPORTANT: The IMU queue carries FIFO-drained batches, not single samples.
    // Each batch preserves IMU sample ordering plus reconstructed sample
    // timestamps at the 416 Hz sensor cadence. The MEKF must therefore step
    // through each sample in the batch sequentially rather than treating the
    // whole batch as one combined update.
    //
    //EKF Update when each queue notification is received
    //Write output to the shared state vector with a timestamp
    //vTaskDelay for a short period to yield to other tasks (e.g.,
    //control task might want to run at a different frequency)

    
    constexpr float kImuPredictDtSec = IMU::kSamplePeriodSeconds;
    Common::IMUBatch      latestImuBatch = {};
    Common::MagData      latestMag   = {};
    Common::BarometerData latestBaro  = {};
    Common::GPSData       latestGps   = {};
    Common::LidarData     latestLidar = {};
    // Keeps the first sample in a new batch time-continuous with the previous batch.
    uint32_t lastImuTimestampMicros = 0;
    bool haveLastImuTimestamp = false;

    float TimestampDeltaSeconds(uint32_t earlierMicros, uint32_t laterMicros) {
        const uint32_t deltaMicros = laterMicros - earlierMicros;
        if (deltaMicros == 0) {
            return kImuPredictDtSec;
        }

        return static_cast<float>(deltaMicros) * 1.0e-6f;
    }
}

void Tasks::NavigationTask(void* pvParameters) {
    pvParameters = pvParameters; // Avoid unused parameter warning
    // TODO: Take the average of multiple samples of mag, accel, and GPS to initialize the EKF state vector
    float magX = 0.0f, magY = 0.0f, magZ = 0.0f;
    float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
    float latitude = 0.0f, longitude = 0.0f, altitude = 0.0f;


    AdaptiveEKF ekf(magX, magY, magZ, accelX, accelY, accelZ, latitude, longitude, altitude); // EKF instance
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

        while(xQueueReceive(SharedData::gI2C2IMUQueue, &latestImuBatch, 0) == pdTRUE) {
            // FIFO transport is batched, but the MEKF still consumes one IMU
            // predict step per sensor sample in time order.

            for (uint8_t i = 0; i < latestImuBatch.count; ++i) {
                float dt = kImuPredictDtSec;
                if (i > 0) {
                    dt = TimestampDeltaSeconds(latestImuBatch.samples[i - 1].timestampMicros,
                                               latestImuBatch.samples[i].timestampMicros);
                } else if (haveLastImuTimestamp) {
                    // The first sample in each batch uses the previous batch tail
                    // so timing stays continuous across FIFO drains.
                    dt = TimestampDeltaSeconds(lastImuTimestampMicros,
                                               latestImuBatch.samples[i].timestampMicros);
                }

                ekf.Predict(latestImuBatch.samples[i], dt);
                lastImuTimestampMicros = latestImuBatch.samples[i].timestampMicros;
                haveLastImuTimestamp = true;
            }
        }
        while(xQueueReceive(SharedData::gI2C1BarometerQueue, &latestBaro, 0) == pdTRUE) {
            ekf.UpdateBarometer(latestBaro);
        }
        while(xQueueReceive(SharedData::gI2C1GPSQueue, &latestGps, 0) == pdTRUE) {
            ekf.UpdateGPS(latestGps);
        }
        while(xQueueReceive(SharedData::gI2C1LidarQueue, &latestLidar, 0) == pdTRUE) {
            ekf.UpdateLidar(latestLidar);
        }
        while(xQueueReceive(SharedData::gI2C2MagQueue, &latestMag, 0) == pdTRUE) {
            ekf.UpdateMagnetometer(latestMag);
        }
    //EKF Predict
    //unlock flight state mutex and read current flight state for adaptive noise tuning
        if(xSemaphoreTake(SharedData::gStateVectorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            SharedData::gStateVector = ekf.GetStateVector(); //TODO: add timestamp potentially?
            xSemaphoreGive(SharedData::gStateVectorMutex);
        }
    
    
        vTaskDelay(pdMS_TO_TICKS(2)); // Adjust delay as needed for desired update rate
    }
}
