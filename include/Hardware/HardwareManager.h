

class HardwareManager{
public:
    void init();
    void read();
    
private:
    float _temperature;
    float _pressure;
    float _baroAltitude;
    float _headingAngle;
    float _accelX;
    float _accelY;
    float _accelZ;

};