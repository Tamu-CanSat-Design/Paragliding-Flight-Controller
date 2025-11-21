#ifndef GNC_H
#define GNC_H

class IController{
public:
    virtual void Reset() =0;
    virtual float Compute(float setpoint, float measurement)=0; // pure virtual
    virtual void set_limits(float min, float max)=0;
};

class IKalmanFilter{
public:
    virtual void Predict() =0;
    virtual void Measure() =0;
    
};



#endif // GNC_H
