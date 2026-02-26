#pragma once
#include <cstddef>

#include <Arduino.h>

namespace Common {

inline constexpr int I2C_BAUDRATE = 400000;

inline constexpr uint16_t TEAM_ID = 1030;
inline constexpr unsigned long TELEMETRY_INTERVAL_MS = 1000;
inline constexpr uint8_t VOLTAGE_PIN = 48;
inline constexpr uint8_t CURRENT_PIN = 49;
inline constexpr uint8_t BAROMETER_PIN = 44;
inline constexpr uint8_t IMU_PIN = 42;
inline constexpr uint8_t GPS_PIN = 50;
inline constexpr uint8_t MAGNETOMETER_PIN = 43;

enum FlightState {
    BOOTUP = 0,
    STANDBY = 1,
    ASCENT = 2,
    DESCENT = 3,
    PAYLOAD_RELEASE = 4,
    RECOVERY = 5
};

inline uint8_t EE_STATE = BOOTUP;


    struct StateVector{
        float x;
        float y;
        float z;
        float vx;
        float vy;
        float vz;
        float q0;
        float q1;
        float q2;
        float q3;
        float q4;
        float accelBias;
        float gyroBias;
    };

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

    struct GPSData {
        float latitude;
        float longitude;
        float altitude;
        float time;
        int satellites;
    };
    
    struct IMUData {
        Vector3 gyroVel;
        Vector3 acceleration;
        Vector3 eulerAngles;
    };

    struct VoltageData {
        float voltage;
        float current;
        float power;
    };

    struct LidarData {
        float distanceMeters;
    };

    struct SensorSnapshot {
        BarometerData barometer;
        IMUData imu;
        GPSData gpsData;
        VoltageData voltage;
       
        LidarData lidarData;
    };



    




    
};