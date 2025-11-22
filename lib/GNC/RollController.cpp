#include <deque>

#define ROLL_KP 1.5f
#define ROLL_KI 0.2f
#define ROLL_KD 0.1f



class RollController{
public:
    void Reset(){
        return;
    }
    float Compute(std::deque<float> SlidingWindow, float dt){

        return 0;
    }
    void SetLimits(float min, float max){
        return;
    }
    float L1Guidance(){
        return 0;
    }
private:
float integral = 0;
};