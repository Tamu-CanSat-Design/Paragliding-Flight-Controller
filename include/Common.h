#include <iostream>

namespace Common{


    const int I2C_BAUDRATE = 400000;

    const uint16_t TEAM_ID = 1030;
    const unsigned long TELEMETRY_INTERVAL_MS = 1000;
    const byte VOLTAGE_PIN = 48;
    const byte CURRENT_PIN = 49;
    const 
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

    
    void ParseCommand(const std::string& command){
        // Simple parser that extracts the command type from a command string
        
    }




    
}