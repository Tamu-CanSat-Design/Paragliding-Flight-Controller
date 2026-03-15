#include "TaskHandler.h"


void Tasks::GuidanceTask(void* pvParameters){
    (void)pvParameters;
    //Guidance logic to determine desired trajectory and attitude setpoints based on current state and mission phase
    //This task will read the current state vector from shared memory, run guidance algorithms, and write desired setpoints back to shared memory for the control task to use.
    while(true){
        //Read current state vector from shared memory (with mutex)
        //Run guidance algorithms to compute desired trajectory and attitude setpoints
        //Write desired setpoints back to shared memory (with mutex)
        vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust delay as needed for guidance update rate
    }
}