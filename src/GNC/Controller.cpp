#include "GNC/Controller.h"

//Descent Controller

DescentController::DescentController(state GainState)
    : DescentGains{{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}}
{
    CurrentState = GainState;
    DescentGains[SLOW] = {1.5f, 0.2f, 0.1f};
    DescentGains[MEDIUM] = {2.0f, 0.3f, 0.15f};
    DescentGains[FAST] = {2.5f, 0.4f, 0.2f};
};

void DescentController::ComputeError(Common::StateVector state, float dt)
{
    float Vel = state.vz;
    error = Vel - 5;
    integral += error * dt;
    derivative = (error - prevError) / dt;
    prevError = error;
};

float DescentController::ComputePID(Common::StateVector state, float dt)
{
    ComputeError(state, dt);

    float PID = DescentGains[CurrentState].KP * error + DescentGains[CurrentState].KI * integral + DescentGains[CurrentState].KD * derivative;
    
    //convert output to degrees for mixer
    PID = PID * (180.0f / PI);
    SetLimits(PID);

    return PID;
};
//Roll Controller

RollController::RollController(state GainState)
    : RollGains{{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}}
{
    CurrentState = GainState;
    RollGains[SLOW] = {1.5f, 0.2f, 0.1f};
    RollGains[MEDIUM] = {2.0f, 0.3f, 0.15f};
    RollGains[FAST] = {2.5f, 0.4f, 0.2f};
};

void RollController::ComputeError(Common::StateVector state, float desiredRoll,float dt, float gyroRollRate)
{
    float qw = state.q0, qx = state.q1, qy = state.q2, qz = state.q3;
    float roll = atan2f(2.0f * (qw*qx + qy*qz), 1.0f - 2.0f * (qx*qx + qy*qy));
    error = desiredRoll - roll;
    integral += error * dt;
    derivative = gyroRollRate;
};

float RollController::ComputePID(Common::StateVector state, float desiredRoll, float dt, float gyroRollRate)
{
    ComputeError(state,desiredRoll,dt, gyroRollRate);
    float PID = RollGains[CurrentState].KP * error + RollGains[CurrentState].KI * integral + RollGains[CurrentState].KD * derivative;
    
    //convert output to degrees for mixer
    PID = PID * (180.0f / PI);
    SetLimits(PID);

    return PID;
};

//Controller Mixer

void ControllerMixer::DetermineState(Common::StateVector stateVector){
    float vx = stateVector.vx, vy = stateVector.vy, vz = stateVector.vz;
    float Vel = sqrt(vz * vz + vx * vx + vy * vy);
    state currentState;
    if (Vel > 0.5f){
        currentState = SLOW;
    }
    else if (Vel > 5.0f){
        currentState = MEDIUM;
    }
    else if (Vel > 10.0f){
        currentState = FAST;
    }
    else{
        currentState = OFF;
    }
    GainState = currentState;
};

void ControllerMixer::UpdateGains(Common::StateVector stateVector){
    DetermineState(stateVector);
    DescentPID.UpdateGains(GainState);
    RollPID.UpdateGains(GainState);
};

void ControllerMixer::Mix(float pitchCmd, float rollCmd)
{
    // Raw mix: Left = Pitch + Roll,  Right = Pitch - Roll
    float rawLeft  = pitchCmd + rollCmd;
    float rawRight = pitchCmd - rollCmd;

    // Clamp to actuator limits [-90, 90] degrees
    leftServoOutput  = SetLimits(rawLeft);
    rightServoOutput = SetLimits(rawRight);
};

void ControllerMixer::CalculateCommands(Common::StateVector stateVector, float desiredRoll, float dt, float gyroRollRate){
    float pitchCmd = DescentPID.ComputePID(stateVector, dt);
    float rollCmd = RollPID.ComputePID(stateVector, desiredRoll, dt, gyroRollRate);
    Mix(pitchCmd, rollCmd);
};