#include <deque>
#define DESCENT_VEL 5.0f
#define DESCENT_KP 1.0f
#define DESCENT_KI 0.1f
#define DESCENT_KD 0.05f


class DescentController{
public:
    void Reset()
    {
        return;
    }

    float ComputePID(std::deque<float> SlidingWindow, float dt) //EKF Descent Velocity Window
    { 
        //Calculate error, integral, and derivative
        //error = measurement - desired vvelcity
        //integral - sum of elements in SlidingWindow queue
        //derivative - difference in two latest queue elements
        
        float error = SlidingWindow.back() - DESCENT_VEL;
        integral += (error - prevError) * dt;
        float derivative = (error-prevError)/dt;
        prevError = error;
        
        float PID = DESCENT_KP * error + DESCENT_KI * integral + DESCENT_KD * derivative;
        SetLimits(PID);        
        return PID;
    }
    void SetLimits(float& PID)
    {
        if(PID > max){
            PID = max;
        }
        else if (PID < min){
            PID = min;
        }
    }
    
    void ActuatorOutputs(float PID){} //idk man

private:
    const float min = -100;
    const float max = 100;
    float integral = 0;
    float prevError = 0;
};