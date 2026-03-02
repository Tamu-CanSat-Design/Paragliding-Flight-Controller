#pragma once

#include <FreeRTOS.h>
#include <semphr.h>

#include "Common.h"

// This file holds any global variables that are shared across multiple tasks.
// All access to these variables should be protected by the corresponding mutex.
namespace SharedData {
// Shared Data structures and Mutexes

    // Shared sensor snapshot and its protecting mutex sent to telemetry
    extern Common::SensorSnapshot gSnapshot;
    extern SemaphoreHandle_t gSnapshotMutex;

    extern Common::FlightState gSD_State;
    extern SemaphoreHandle_t gSD_StateMutex;

    extern Common::StateVector gStateVector;
    extern SemaphoreHandle_t gStateVectorMutex;

// Shared queues
    //I2C 1 queues to Nav
    extern QueueSetHandle_t gI2C1QueueSet; // GPS, Barometer, Lidar
    extern QueueHandle_t gI2C1BarometerQueue;
    extern QueueHandle_t gI2C1GPSQueue;
    extern QueueHandle_t gI2C1LidarQueue;
    //I2C 2 queues to Nav
    extern QueueSetHandle_t gI2C2QueueSet; // IMU, Magnetometer, Voltage
    extern QueueHandle_t gI2C2IMUQueue;
    extern QueueHandle_t gI2C2MagQueue;
    extern QueueHandle_t gI2C2VoltageQueue;

    //Command queue
    extern QueueHandle_t gCommandQueue;

    void InitializeThreadSafeData();
}