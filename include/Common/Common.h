#include <iostream>

namespace Common{
    struct Vector3{
        float X;
        float Y;
        float Z;
    };

    struct telemetry{
        const int teamID;
        float missionTime;
        int packetCount;
        char Mode;
        std::string state;
        float baroAltitude;
        float temperature;
        float pressure;
        float voltage;
        float current;
        Vector3 gyroVel;   
        Vector3 acceleration;      
        float GPSTime;
        Vector3 GPSLoc;
        int GPSSats;
        std::string CMDEcho; 
        Vector3 eulerAngles;
        Vector3 velocity;
        Vector3 position;
    };
}