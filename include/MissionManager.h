#ifndef __MISSION_MANAGER_H__
#define __MISSION_MANAGER_H__
#include "Common.h"
#include "Hardware/Hardware.h"
// State logic

namespace MissionManager {


  extern char Command[Common::COMMAND_BUFFER_LENGTH];

  
  
  extern bool CXFlag;
  extern bool LDARFlag;
  extern bool EGRELFlag;
  extern bool AUDFlag; 

  extern bool AscentStartedFlag;
  extern bool DescentStartedFlag;
  extern bool PayloadReleaseStartedFlag;
  extern bool RecoveryStartedFlag;
  extern float MaxAltitude;


  void CAL();
  void EGREL();
  void LDAR();


  Common::FlightState CheckStateChange(Common::FlightState currentState, Common::StateVector ekfState, Common::SensorSnapshot sensorSnapshot);





}




#endif // __MISSION_MANAGER_H__


