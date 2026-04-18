#include "GNC/Controller.h"


MixerOutput ControllerMixer::Mix(float pitchCmd, float rollCmd){
    
    // Raw mix: Left = Pitch + Roll,  Right = Pitch - Roll
    float rawLeft  = pitchCmd + rollCmd;
    float rawRight = pitchCmd - rollCmd;

    // Clamp to actuator limits [0, 100]
    ControllerOutput.left  = Clamp(rawLeft,  0.0f, 100.0f);
    ControllerOutput.right = Clamp(rawRight, 0.0f, 100.0f);

    return ControllerOutput;
};

MixerOutput ControllerMixer::Compute(float descentPID, float rollPID)
{
    float pitchCmd = 50.0f + descentPID * 0.5f;
    pitchCmd = Clamp(pitchCmd, 0.0f, 100.0f);

    float rollCmd = rollPID * 0.5f;

    return Mix(pitchCmd, rollCmd);
}
