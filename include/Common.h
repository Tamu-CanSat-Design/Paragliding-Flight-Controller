#include <iostream>
namespace Common{


    const int I2C_BAUDRATE = 400000;

    const uint16_t TEAM_ID = 1030;
    const unsigned long TELEMETRY_INTERVAL_MS = 1000;
    const uint8_t VOLTAGE_PIN = 48;
    const uint8_t CURRENT_PIN = 49;
    const uint8_t BAROMETER_PIN = 44;
    const uint8_t IMU_PIN = 42;
    const uint8_t GPS_PIN = 50;
    const uint8_t MAGNETOMETER_PIN = 43;

    
    enum FlightState {
        BOOTUP = 0,
        STANDBY = 1,
        ASCENT = 2,
        DESCENT = 3,
        PAYLOAD_RELEASE = 4,
        RECOVERY = 5
    };

    FlightState EE_STATE = BOOTUP;

    struct Vector3{
        float x;
        float y;
        float z;
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
                             ",GyroX:" + std::to_string(data.gyroVel.x) +
                             ",GyroY:" + std::to_string(data.gyroVel.y) +
                             ",GyroZ:" + std::to_string(data.gyroVel.z) +
                             ",AccX:" + std::to_string(data.acceleration.x) +
                             ",AccY:" + std::to_string(data.acceleration.y) +
                             ",AccZ:" + std::to_string(data.acceleration.z) +
                             ",GPSTime:" + std::to_string(data.GPSTime) +
                             ",GPSLat:" + std::to_string(data.GPSLoc.x) +
                             ",GPSLon:" + std::to_string(data.GPSLoc.y) +
                             ",GPSSats:" + std::to_string(data.GPSSats) +
                             ",CMDEcho:" + data.CMDEcho +
                             ",EulerX:" + std::to_string(data.eulerAngles.x) +
                             ",EulerY:" + std::to_string(data.eulerAngles.y) +
                             ",EulerZ:" + std::to_string(data.eulerAngles.z) +
                             ",VelX:" + std::to_string(data.velocity.x) +
                             ",VelY:" + std::to_string(data.velocity.y) +
                             ",VelZ:" + std::to_string(data.velocity.z) +
                             ",PosX:" + std::to_string(data.position.x) +
                             ",PosY:" + std::to_string(data.position.y) +
                             ",PosZ:" + std::to_string(data.position.z);
        return packet;
    }
    
    void ParseCommand(const std::string& command){
        // Simple parser that extracts the command type from a command string
        
    }




    
}