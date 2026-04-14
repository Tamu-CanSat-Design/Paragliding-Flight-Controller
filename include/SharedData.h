#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <FreeRTOS.h>
#include <semphr.h>

#include "Common.h"
#include "Commlink.h"
// This file holds any global variables that are shared across multiple tasks.
// All access to these variables should be protected by the corresponding mutex.
namespace SharedData {
// Shared Data structures and Mutexes

    // Shared sensor snapshot and its protecting mutex sent to telemetry
    extern Common::SensorSnapshot gSnapshot;
    extern SemaphoreHandle_t gSnapshotMutex;

    extern Common::FlightState gFlightState;
    extern SemaphoreHandle_t gFlightStateMutex;

    extern bool gSimEnabled;
    extern SemaphoreHandle_t gSimEnabledMutex;

    extern bool gCXFlag;
    extern SemaphoreHandle_t gCXFlagMutex;

    extern float gReferencePressureHpa;
    extern SemaphoreHandle_t gReferencePressureMutex;

    extern Common::StateVector gStateVector;
    extern SemaphoreHandle_t gStateVectorMutex;

    extern char gCommand[Common::COMMAND_BUFFER_LENGTH];
    extern SemaphoreHandle_t gCommandMutex;

// Shared queues
    //I2C 1 queues to Nav
    extern QueueHandle_t gI2C1BarometerQueue;
    extern QueueHandle_t gI2C1GPSQueue;
    extern QueueHandle_t gI2C1LidarQueue;
    //I2C 2 queues to Nav
    extern QueueHandle_t gI2C2IMUQueue;
    extern QueueHandle_t gI2C2MagQueue;
    extern QueueHandle_t gI2C2VoltageQueue;

    //Command queue
    extern QueueHandle_t gCommandQueue;

    extern Commlink commlink;

    void InitializeThreadSafeData();
}

#endif // SHARED_DATA_H
