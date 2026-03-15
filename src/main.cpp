#ifdef SENSOR_TESTBENCH

#include "Testbench/Testbench.h"

extern "C" void setup() {
  Testbench::Setup();
}

extern "C" void loop() {
  Testbench::Loop();
}

#else

#include "TaskHandler.h"

namespace {
constexpr const char* kBuildDate = __DATE__;
constexpr const char* kBuildTime = __TIME__;
}

extern "C" void setup() {
  Serial.begin(115200);
  Serial.printf("Build: %s %s\n", kBuildDate, kBuildTime);

  // Initialize all sensors before proceeding.
  while (!(Hardware::InitializeAllSensors())) {
    // Optionally add a delay or error handling here.
  }

  // Create tasks and initialize hardware before starting the scheduler.
  Tasks::InitializeTasks();

  // Start the scheduler if it is not already running.
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    vTaskStartScheduler();
  }
}

extern "C" void loop() {
  // The scheduler should take over; loop stays as a low-priority idle fallback.
}

#endif
