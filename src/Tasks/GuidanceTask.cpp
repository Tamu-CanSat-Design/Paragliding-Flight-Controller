#include "TaskHandler.h"
#include "GNC/Guidance.h"
#include "GNC/PathPlanner.h"

namespace{
    Guidance guidance;
    PathPlanner pathPlanner;
    Common::StateVector latestStateVector;
    Common::DropZone latestDropZone;
    Common::Vector3 latestWaypoint = {0.0f, 0.0f, 0.0f};
    float rollAngle = 0.0f;
}

void Tasks::GuidanceTask(void* pvParameters){
    (void)pvParameters;
    //Guidance logic to determine desired trajectory and attitude setpoints based on current state and mission phase
    //This task will read the current state vector from shared memory, run guidance algorithms, and write desired setpoints back to shared memory for the control task to use.
    while(true){
        //Read current state vector from shared memory (with mutex)

        if(xSemaphoreTake(SharedData::gStateVectorMutex, 0) == pdTRUE) {
            latestStateVector = SharedData::gStateVector;
            xSemaphoreGive(SharedData::gStateVectorMutex);
        }
        else{}

        latestDropZone = {0.0f, 0.0f, 0.0f}; 

        //Read current drop zone from shared memory (with mutex)
        //Assuming we have a shared variable for the drop zone, which may need to be         
        
        //Run path planner to get desired waypoint based on current state and drop zone
        pathPlanner.UpdateState(latestStateVector, latestWaypoint);
        latestWaypoint = pathPlanner.CalculateWaypoint(latestDropZone);

        //Run L1 guidance to get desired roll angle based on current state and desired waypoint
        //Ansh can you check this
        rollAngle = guidance.CalculateRoll(latestStateVector, latestWaypoint);
        if(xSemaphoreTake(SharedData::gRollAngleSetpointMutex, 0) == pdTRUE) {
            SharedData::gRollAngleSetpoint = rollAngle;
            xSemaphoreGive(SharedData::gRollAngleSetpointMutex);
        }
        

        //Run guidance algorithms to compute desired trajectory and attitude setpoints
        //Write desired setpoints back to shared memory (with mutex)
        vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust delay as needed for guidance update rate
    }
}