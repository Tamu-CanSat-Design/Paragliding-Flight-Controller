#include "TaskHandler.h"



void setup() {
  Serial.begin(115200);

  // Initialize all sensors before proceeding.
  while(!(Hardware::InitializeAllSensors())){
    // Optionally add a delay or error handling here
    
  }

  // Create tasks and initialize hardware before starting the scheduler.
  
  Tasks::InitializeTasks();
  

  Serial.println("All tasks initialized. Starting scheduler...");
  // Start the scheduler if it is not already running.
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    vTaskStartScheduler();
  }
}

void loop() {
  // The scheduler should take over; loop stays as a low-priority idle fallback.
  }
