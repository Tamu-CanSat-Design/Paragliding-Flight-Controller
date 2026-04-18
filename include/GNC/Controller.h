#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Common.h"
#include "GNC/GNC.h"

struct MixerOutput{
    float left;
    float right;
};

class RollController{
    private:
        float Roll_KP = 1.5f;
        float Roll_KI = 0.2f;
        float Roll_KD = 0.1f;
        const float max = 10;
        const float min = -10;
        float integral = 0;
        float prevError = 0;
        void SetLimits(float& PID){
            if (PID > max){
                PID = max;
            }
            else if (PID < min){
                PID = min;
            }
            return;
        };
    public:


};

class DescentController{
    private:
        float Roll_KP = 1.5f;
        float Roll_KI = 0.2f;
        float Roll_KD = 0.1f;

        const float min = -100;
        const float max = 100;
        float integral = 0;
        float prevError = 0;

        void SetLimits(float& PID)
        {
            if(PID > max){
                PID = max;
            }
            else if (PID < min){
                PID = min;
            }
        };

    public:

        void Reset();
        float ComputePID(float Vel, float dt);

};

class ControllerMixer{
    private:
        MixerOutput ControllerOutput;
        static float Clamp(float v, float lo, float hi) {
            if (v < lo) return lo;
            if (v > hi) return hi;
            return v;
        }

    public:

        MixerOutput Mix(float pitchCmd, float rollCmd);
             
        MixerOutput Compute(float descentPID, float rollPID);
};

#endif // CONTROLLER_H