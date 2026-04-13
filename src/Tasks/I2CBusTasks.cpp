#include "TaskHandler.h"

#include <cmath>

namespace {
constexpr float kDefaultReferencePressureHpa = 1013.25f;

float ReadReferencePressureHpa() {
    float referencePressure = kDefaultReferencePressureHpa;

    if (SharedData::gReferencePressureMutex != nullptr
        && xSemaphoreTake(SharedData::gReferencePressureMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        referencePressure = SharedData::gReferencePressureHpa;
        xSemaphoreGive(SharedData::gReferencePressureMutex);
    }

    return referencePressure;
}

float PressureToRelativeAltitude(float pressureHpa, float referencePressureHpa) {
    if (pressureHpa <= 0.0f || referencePressureHpa <= 0.0f) {
        return 0.0f;
    }

    const float ratio = pressureHpa / referencePressureHpa;
    return 44330.0f * (1.0f - std::pow(ratio, 0.190294957f));
}

template <typename T>
void UpdateSnapshot(T Common::SensorSnapshot::* field, const T& value) {
    if (SharedData::gSnapshotMutex == nullptr) {
        return;
    }

    if (xSemaphoreTake(SharedData::gSnapshotMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        SharedData::gSnapshot.*field = value;
        xSemaphoreGive(SharedData::gSnapshotMutex);
    }
}
}  // namespace

void Tasks::I2C1BusTask(void* pvParameters) {
    (void)pvParameters;

    Common::BarometerData latestBaro = {};
    Common::GPSData latestGps = {};
    Common::LidarData latestLidar = {};

    while (true) {
        latestBaro = Hardware::barometer_.ReadData();
        latestBaro.altitude = PressureToRelativeAltitude(latestBaro.pressure, ReadReferencePressureHpa());

        latestGps = Hardware::gps_.ReadData();
        latestLidar = Hardware::lidar_.ReadData();

        xQueueSend(SharedData::gI2C1BarometerQueue, &latestBaro, portMAX_DELAY);
        xQueueSend(SharedData::gI2C1GPSQueue, &latestGps, portMAX_DELAY);
        xQueueSend(SharedData::gI2C1LidarQueue, &latestLidar, portMAX_DELAY);

        UpdateSnapshot(&Common::SensorSnapshot::barometerData, latestBaro);
        UpdateSnapshot(&Common::SensorSnapshot::gpsData, latestGps);
        UpdateSnapshot(&Common::SensorSnapshot::lidarData, latestLidar);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Tasks::I2C2BusTask(void* pvParameters) {
    (void)pvParameters;

    Common::IMUData latestImu = {};
    Common::MagData latestMag = {};
    Common::VoltageData latestVoltage = {};

    while (true) {
        latestImu = Hardware::imu_.ReadData();
        latestVoltage = Hardware::voltageSensor_.ReadData();

        xQueueSend(SharedData::gI2C2IMUQueue, &latestImu, portMAX_DELAY);
        xQueueSend(SharedData::gI2C2MagQueue, &latestMag, portMAX_DELAY);
        xQueueSend(SharedData::gI2C2VoltageQueue, &latestVoltage, portMAX_DELAY);

        UpdateSnapshot(&Common::SensorSnapshot::imuData, latestImu);
        UpdateSnapshot(&Common::SensorSnapshot::magData, latestMag);
        UpdateSnapshot(&Common::SensorSnapshot::voltageData, latestVoltage);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
