#include <iostream>

namespace Common{


    const int I2C_BAUDRATE = 400000;

    const uint16_t TEAM_ID = 1030;
    const unsigned long TELEMETRY_INTERVAL_MS = 1000;
    const byte VOLTAGE_PIN = 48;
    const byte CURRENT_PIN = 49;
    const byte BAROMETER_PIN = 44;
    const byte IMU_PIN = 42;
    const byte GPS_PIN = 50;
    const byte MAGNETOMETER_PIN = 43;

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

    std::string BuildPacket(const telemetry& data){ //packet is innaccurate here (copilot made this)
        // Simple packet builder that converts telemetry data to a string format
        std::string packet = "ID:" + std::to_string(data.teamID) +
                             ",Time:" + std::to_string(data.missionTime) +
                             ",PacketCount:" + std::to_string(data.packetCount) +
                             ",Mode:" + data.Mode +
                             ",State:" + data.state +
                             ",BaroAlt:" + std::to_string(data.baroAltitude) +
                             ",Temp:" + std::to_string(data.temperature) +
                             ",Pressure:" + std::to_string(data.pressure) +
                             ",Voltage:" + std::to_string(data.voltage) +
                             ",Current:" + std::to_string(data.current) +
                             ",GyroX:" + std::to_string(data.gyroVel.X) +
                             ",GyroY:" + std::to_string(data.gyroVel.Y) +
                             ",GyroZ:" + std::to_string(data.gyroVel.Z) +
                             ",AccX:" + std::to_string(data.acceleration.X) +
                             ",AccY:" + std::to_string(data.acceleration.Y) +
                             ",AccZ:" + std::to_string(data.acceleration.Z) +
                             ",GPSTime:" + std::to_string(data.GPSTime) +
                             ",GPSLat:" + std::to_string(data.GPSLoc.X) +
                             ",GPSLon:" + std::to_string(data.GPSLoc.Y) +
                             ",GPSSats:" + std::to_string(data.GPSSats) +
                             ",CMDEcho:" + data.CMDEcho +
                             ",EulerX:" + std::to_string(data.eulerAngles.X) +
                             ",EulerY:" + std::to_string(data.eulerAngles.Y) +
                             ",EulerZ:" + std::to_string(data.eulerAngles.Z) +
                             ",VelX:" + std::to_string(data.velocity.X) +
                             ",VelY:" + std::to_string(data.velocity.Y) +
                             ",VelZ:" + std::to_string(data.velocity.Z) +
                             ",PosX:" + std::to_string(data.position.X) +
                             ",PosY:" + std::to_string(data.position.Y) +
                             ",PosZ:" + std::to_string(data.position.Z);
        return packet;
    }
    
    void ParseCommand(const std::string& command){
        // Simple parser that extracts the command type from a command string
        
    }




    
}