#include <Arduino.h>



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


  // Start Tasks


  // Sensor Tasks:
  // Begin I2C
  

  // Initialize BMP390 and remember calibration data
  // Initialize IMU
  // Initialize GPS
  // Initialize Current Sensor
  // Initialize Voltage Sensor
 
  //Create mutex for all sensor data





  //Watchdog Setup
  // 



  // Telemetry Setup
  // Initialize Radio
  // Initialize Telemetry Packet Structure




  //Initialize Queues for inter-task communication





  

}

void loop() {
  // put your main code here, to run repeatedly:

  // switch (States::EE_STATE)
  // { 
  //   case 1:
  //     States::Ascent();
  //     break;
  //   case 2:
  //     States::StartDescent();
  //     break;
  //   case 3:
  //     States::PayloadRelease();
  //     break;    
  //   case 4:
  //     States::Landing();
  //     break;
  //   case 5:
  //     States::Recovery();
  //     break;
  //   default:
  //     States::Ascent();
  //     break;
  // }
  // delay(Common::TELEMETRY_DELAY);

}

