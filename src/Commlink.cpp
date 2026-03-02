#include "Commlink.h"
#include <Arduino.h>
#include <cstdio>
#include "Common.h"
void Commlink::Initialize(){
    Serial.begin(SERIAL_BAUDRATE);
    
}

struct telemetry {  // This struct needs to be reordered for memory optimization
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
    Common::Vector3 gyroVel;
    Common::Vector3 acceleration;
    float GPSTime;
    Common::Vector3 GPSLoc;
    int GPSSats;
    char CMDEcho[32];
    Common::Vector3 eulerAngles;
    Common::Vector3 velocity;
    Common::Vector3 position;
};

int BuildPacket(const telemetry& data, char* buffer, size_t bufferSize){ // packet format is placeholder
        // Simple packet builder that writes into a fixed buffer (no heap).
        return std::snprintf(
            buffer,
            bufferSize,
            "ID:%d,Time:%f,PacketCount:%d,Mode:%c,State:%s,BaroAlt:%f,Temp:%f,Pressure:%f,"
            "Voltage:%f,Current:%f,GyroX:%f,GyroY:%f,GyroZ:%f,AccX:%f,AccY:%f,AccZ:%f,GPSTime:%f,"
            "GPSLat:%f,GPSLon:%f,GPSAlt:%f,GPSSats:%d,CMDEcho:%s,EulerX:%f,EulerY:%f,EulerZ:%f,VelX:%f,VelY:%f,VelZ:%f,PosX:%f,PosY:%f,PosZ:%f",
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
            data.GPSLoc.x,
            data.GPSLoc.y,
            data.GPSLoc.z,
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