#include <cstddef>
#include <cstdio>
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

    struct BarometerData {
        float pressure;
        float temperature;
        float altitude;
    };

    struct GpsData {
        float latitude;
        float longitude;
        float altitude;
    };

    struct telemetry{
        const int teamID;
        float missionTime;
        int packetCount;
        char Mode;
        char state[16];
        float baroAltitude;
        float temperature;
        float pressure;
        float voltage;
        float current;
        Vector3 gyroVel;   
        Vector3 acceleration;      
        float GPSTime;
        GpsData GPSLoc;
        int GPSSats;
        char CMDEcho[32];
        Vector3 eulerAngles;
        Vector3 velocity;
        Vector3 position;
    };

    int BuildPacket(const telemetry& data, char* buffer, size_t bufferSize){ // packet format is placeholder
        // Simple packet builder that writes into a fixed buffer (no heap).
        return std::snprintf(
            buffer,
            bufferSize,
            "ID:%d,Time:%f,PacketCount:%d,Mode:%c,State:%s,BaroAlt:%f,Temp:%f,Pressure:%f,Voltage:%f,Current:%f,"
            "GyroX:%f,GyroY:%f,GyroZ:%f,AccX:%f,AccY:%f,AccZ:%f,GPSTime:%f,GPSLat:%f,GPSLon:%f,GPSSats:%d,"
            "CMDEcho:%s,EulerX:%f,EulerY:%f,EulerZ:%f,VelX:%f,VelY:%f,VelZ:%f,PosX:%f,PosY:%f,PosZ:%f",
            data.teamID,
            data.missionTime,
            data.packetCount,
            data.Mode,
            data.state,
            data.baroAltitude,
            data.temperature,
            data.pressure,
            data.voltage,
            data.current,
            data.gyroVel.x,
            data.gyroVel.y,
            data.gyroVel.z,
            data.acceleration.x,
            data.acceleration.y,
            data.acceleration.z,
            data.GPSTime,
            data.GPSLoc.latitude,
            data.GPSLoc.longitude,
            data.GPSSats,
            data.CMDEcho,
            data.eulerAngles.x,
            data.eulerAngles.y,
            data.eulerAngles.z,
            data.velocity.x,
            data.velocity.y,
            data.velocity.z,
            data.position.x,
            data.position.y,
            data.position.z
        );
    }
    
    void ParseCommand(const char* command){
        // Simple parser that extracts the command type from a command string
        
    }




    
}
