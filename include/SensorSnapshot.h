#ifndef INCLUDE_SENSOR_SNAPSHOT_H
#define INCLUDE_SENSOR_SNAPSHOT_H

#include "Common.h"
#include "interfaces/IMU.h"

struct SensorSnapshot {
    Common::BarometerData barometer;
    IMUData imu;
    Common::GpsData gpsLocation;
    float gpsTime = 0.0f;
    int gpsSatellites = 0;
    float rangeMeters = 0.0f;
    float voltage = 0.0f;
    float current = 0.0f;
};

#endif  // INCLUDE_SENSOR_SNAPSHOT_H
