#include <deque>

#define ROLL_KP 1.5f
#define ROLL_KI 0.2f
#define ROLL_KD 0.1f



class RollController{
public:
    void Reset(){
        integral = 0;
        prevError = 0;
        return;
    }
    float ComputePID(float roll, float rollCommand, float dt){
        //Calculate error, integral, and derivative
        //error = measurement - desired velocity
        //integral - sum of elements in SlidingWindow queue
        //derivative - difference in two latest queue elements
        
        float error = roll - rollCommand;
        integral += (error - prevError) * dt;
        float derivative = (error-prevError)/dt;
        prevError = error;
        
        float PID = ROLL_KP * error + ROLL_KI * integral + ROLL_KD * derivative;
        SetLimits(PID);        
        return PID;
        return 0;
    }
    void SetLimits(float& PID){
        if (PID > max){
            PID = max;
        }
        else if (PID < min){
            PID = min;
        }
        return;
    }
    float L1Guidance(){
        return 0;
    }
private:
    const float max = 100;
    const float min = 100;
    float integral = 0;
    float prevError = 0;
};