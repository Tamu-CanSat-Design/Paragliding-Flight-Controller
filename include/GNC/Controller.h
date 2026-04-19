#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Common.h"
#include "GNC/GNC.h"

struct MixerOutput{
    float left;
    float right;
};

struct Gains{
    float KP;
    float KD;
    float KI;
};

enum state{off, slow, med, fast};

class RollController{
    private:
        state CurrentState = off;
        Gains RollGains[4]; 
        float error = 0;
        const float max = 60;
        const float min = -60;
        float integral = 0;
        float derivative = 0;
        
        inline void SetLimits(float& PID){
            if (PID > max){
                PID = max;
            }
            else if (PID < min){
                PID = min;
            }
            return;
        };
    public:
        RollController(state GainState);
        inline void Reset(){
            integral = 0;
            derivative = 0;
        };

        inline void UpdateGains(state GainState){
            CurrentState = GainState; 
        };

        void ComputeError(Common::StateVector state, float desiredRoll,float dt);
        float ComputePID(Common::StateVector state, float desiredRoll, float dt);


};

class DescentController{
    private:
        state CurrentState = off;
        Gains DescentGains[4];
        const float min = -40;
        const float max = 40;
        float integral = 0;
        float error = 0;
        float prevError = 0;
        float derivative = 0;

        inline void SetLimits(float& PID)
        {
            if(PID > max){
                PID = max;
            }
            else if (PID < min){
                PID = min;
            }
        };

        void DeadBand();

    public:
        DescentController(state GainState);
        
        inline void Reset(){
            integral = 0;
            prevError = 0;
        };

        inline void UpdateGains(state GainState){
            CurrentState = GainState;
        };

        void ComputeError(Common::StateVector state, float dt);
        float ComputePID(Common::StateVector state, float dt);

};

class ControllerMixer{
    private:
        state GainState;

        DescentController DescentPID;
        RollController RollPID;

        const float max = 90.0f;
        const float min = -90.0f;
        MixerOutput ControllerOutput = {0,0};

        float SetLimits(float& PID)
        {
            if(PID > max){
                PID = max;
            }
            else if (PID < min){
                PID = min;
            }
            return PID;
        };

    public:

        ControllerMixer():GainState(off), DescentPID(GainState), RollPID(GainState){};

        void DetermineState(Common::StateVector stateVector);
        void UpdateGains(Common::StateVector stateVector);

        void Mix(float pitchCmd, float rollCmd);
        void CalculateCommands(Common::StateVector stateVector, float desiredRoll, float dt);
        
        inline float GetLeftOutput(){
            return ControllerOutput.left;
        };
        inline float GetRightOutput(){
            return ControllerOutput.right;
        };
};

#endif // CONTROLLER_H