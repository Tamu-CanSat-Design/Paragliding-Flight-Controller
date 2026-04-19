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

enum state{
    OFF = 0,
    SLOW = 1,
    MEDIUM = 2,
    FAST = 3
};

class RollController{
    private:
        state CurrentState = OFF;
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

        void ComputeError(Common::StateVector state, float desiredRoll,float dt, float gyroRollRate);
        float ComputePID(Common::StateVector state, float desiredRoll, float dt, float gyroRollRate);


};

class DescentController{
    private:
        state CurrentState = OFF;
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

        const float maxAngle = 90.0f;
        const float minAngle = -90.0f;

        float leftServoOutput;
        float rightServoOutput;

        float SetLimits(float& PID)
        {
            if(PID > maxAngle){
                PID = maxAngle;
            }
            else if (PID < minAngle){
                PID = minAngle;
            }
            return PID;
        };

    public:

        ControllerMixer():GainState(OFF), DescentPID(GainState), RollPID(GainState){};

        void DetermineState(Common::StateVector stateVector);
        void UpdateGains(Common::StateVector stateVector);

        void Mix(float pitchCmd, float rollCmd);
        void CalculateCommands(Common::StateVector stateVector, float desiredRoll, float dt, float gyroRollRate);
        
        inline float GetLeftOutput(){
            return leftServoOutput;
        };
        inline float GetRightOutput(){
            return rightServoOutput;
        };
};

#endif // CONTROLLER_H