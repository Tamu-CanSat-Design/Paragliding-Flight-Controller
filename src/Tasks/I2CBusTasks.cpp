#include "TaskHandler.h"

#include <cmath>

namespace {
uint32_t CaptureTimestampMs() {
    return millis();
}

void SendI2C1ControlResult(Common::I2C1ControlCommandType type, bool success) {
    Common::I2C1ControlResult result{};
    result.type = type;
    result.success = success;
    xQueueSend(SharedData::gI2C1ControlResultQueue, &result, portMAX_DELAY);
}

// The I2C1 task owns the barometer, so maintenance actions such as rezeroing
// are routed here from mission logic instead of touching the driver cross-task.
bool HandleI2C1ControlCommands() {
    bool rezeroRequested = false;
    Common::I2C1ControlCommand command{};
    while (xQueueReceive(SharedData::gI2C1ControlQueue, &command, 0) == pdTRUE) {
        switch (command.type) {
            case Common::I2C1ControlCommandType::kRezeroBarometer:
                rezeroRequested = true;
                break;
            default:
                SendI2C1ControlResult(command.type, false);
                break;
        }
    }

    return rezeroRequested;
}

// Update a single field of gSnapshot without touching fields owned by other tasks.
// Uses a non-blocking take so the bus task never stalls on a slow telemetry reader.
template<typename Field, typename Value>
void UpdateSnapshot(Field Common::SensorSnapshot::* field, const Value& value) {
    if (xSemaphoreTake(SharedData::gSnapshotMutex, 0) == pdTRUE) {
        SharedData::gSnapshot.*field = value;
        xSemaphoreGive(SharedData::gSnapshotMutex);
    }
}
}


void Tasks::I2C1BusTask(void* pvParameters) {
    (void)pvParameters;

    Common::BarometerData latestBaro  = {};
    Common::GPSData       latestGps   = {};
    Common::LidarData     latestLidar = {};

    // Target rates: barometer 50 Hz, lidar 20 Hz, GPS 5 Hz.
    constexpr TickType_t kBaroPeriod  = pdMS_TO_TICKS(20);  // 50 Hz
    constexpr TickType_t kLidarPeriod = pdMS_TO_TICKS(50);  // 20 Hz
    constexpr TickType_t kGpsPeriod   = pdMS_TO_TICKS(200); // 5 Hz
    constexpr TickType_t kLidarTimeout = pdMS_TO_TICKS(40);

    TickType_t lastBaroTick  = 0;
    TickType_t lastLidarTick = 0;
    TickType_t lastGpsTick   = 0;
    TickType_t lidarMeasurementStartTick = 0;
    bool lidarMeasurementPending = false;
    bool barometerCalCommandPending = false;

    while (true) {
        HealthMonitor::Kick(HealthMonitor::TaskId::I2C1Bus);

        // Process pending control actions before publishing the next sensor sample.
        if (HandleI2C1ControlCommands()) {
            Hardware::barometer_.StartReferencePressureCapture(CaptureTimestampMs());
            barometerCalCommandPending = true;
        }

        const TickType_t now = xTaskGetTickCount();

        if ((now - lastBaroTick) >= kBaroPeriod) {
            // In SIM mode, substitute the ground-commanded pressure for the real read.
            bool simEnabled = false;
            if (xSemaphoreTake(SharedData::gSimEnabledMutex, 0) == pdTRUE) {
                simEnabled = SharedData::gSimEnabled;
                xSemaphoreGive(SharedData::gSimEnabledMutex);
            }

            if (simEnabled) {
                float simPressure = 0.0f;
                if (xSemaphoreTake(SharedData::gSimPressureMutex, 0) == pdTRUE) {
                    simPressure = SharedData::gSimPressureHpa;
                    xSemaphoreGive(SharedData::gSimPressureMutex);
                }
                if (simPressure > 0.0f) {
                    const float refPressure = Hardware::barometer_.GetReferencePressureHpa();
                    latestBaro.pressure = simPressure;
                    latestBaro.altitude = (refPressure > 0.0f)
                        ? 44330.0f * (1.0f - powf(simPressure / refPressure, 1.0f / 5.255f))
                        : 0.0f;
                    // Temperature retains the last real value.
                }
            } else {
                latestBaro = Hardware::barometer_.ReadData();
            }
            latestBaro.timestampMs = CaptureTimestampMs();

            xQueueSend(SharedData::gI2C1BarometerQueue, &latestBaro, 0);
            UpdateSnapshot(&Common::SensorSnapshot::barometerData, latestBaro);

            Common::BarometerData captureSample{};
            bool hasCaptureSample = false;
            if (Hardware::barometer_.ReferencePressureCaptureInProgress()) {
                if (simEnabled) {
                    captureSample = Hardware::barometer_.ReadData();
                    if (Hardware::barometer_.LastReadSucceeded()) {
                        captureSample.timestampMs = latestBaro.timestampMs;
                        hasCaptureSample = true;
                    }
                } else if (Hardware::barometer_.LastReadSucceeded()) {
                    captureSample = latestBaro;
                    hasCaptureSample = true;
                }

                const Common::BarometerData sampleForCapture = hasCaptureSample
                    ? captureSample
                    : Common::BarometerData{};
                const auto captureStatus = Hardware::barometer_.ProcessReferencePressureSample(
                    sampleForCapture, latestBaro.timestampMs);
                if (barometerCalCommandPending) {
                    if (captureStatus == Barometer::ReferenceCaptureStatus::kReady) {
                        SendI2C1ControlResult(Common::I2C1ControlCommandType::kRezeroBarometer, true);
                        barometerCalCommandPending = false;
                    } else if (captureStatus == Barometer::ReferenceCaptureStatus::kTimedOut) {
                        SendI2C1ControlResult(Common::I2C1ControlCommandType::kRezeroBarometer, false);
                        barometerCalCommandPending = false;
                    }
                }
            }

            lastBaroTick = now;
        }

        if (!lidarMeasurementPending && (now - lastLidarTick) >= kLidarPeriod) {
            if (Hardware::lidar_.StartMeasurement(true)) {
                lidarMeasurementPending = true;
                lidarMeasurementStartTick = now;
            }
            lastLidarTick = now;
        }

        if (lidarMeasurementPending) {
            Common::LidarData completedLidar = latestLidar;
            const auto lidarStatus = Hardware::lidar_.ReadMeasurementIfReady(&completedLidar);
            if (lidarStatus == Lidar::MeasurementStatus::kReady) {
                completedLidar.timestampMs = CaptureTimestampMs();
                latestLidar = completedLidar;
                xQueueSend(SharedData::gI2C1LidarQueue, &latestLidar, 0);
                UpdateSnapshot(&Common::SensorSnapshot::lidarData, latestLidar);
                lidarMeasurementPending = false;
            } else if (lidarStatus == Lidar::MeasurementStatus::kError
                       || (now - lidarMeasurementStartTick) >= kLidarTimeout) {
                lidarMeasurementPending = false;
            }
        }

        if ((now - lastGpsTick) >= kGpsPeriod) {
            latestGps = Hardware::gps_.ReadData();
            latestGps.timestampMs = CaptureTimestampMs();
            xQueueSend(SharedData::gI2C1GPSQueue, &latestGps, 0);
            UpdateSnapshot(&Common::SensorSnapshot::gpsData, latestGps);
            lastGpsTick = now;
        }

        // 5 ms loop granularity gives 200 Hz scheduling resolution.
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void Tasks::I2C2BusTask(void* pvParameters) {
    (void)pvParameters;

    Common::IMUData     latestImu     = {};
    Common::MagData     latestMag     = {};
    Common::VoltageData latestVoltage = {};

    // Target rates: IMU+mag ~100 Hz, voltage 7 Hz.
    constexpr TickType_t kImuPeriod     = pdMS_TO_TICKS(10);  // ~100 Hz (LSM6DSR ODR is 104 Hz)
    constexpr TickType_t kVoltagePeriod = pdMS_TO_TICKS(143); // ~7 Hz

    TickType_t lastImuTick     = 0;
    TickType_t lastVoltageTick = 0;

    while (true) {
        HealthMonitor::Kick(HealthMonitor::TaskId::I2C2Bus);

        const TickType_t now = xTaskGetTickCount();

        if (Hardware::imu_.CalibrationInProgress()) {
            Hardware::imu_.CalibrationStep();
        }

        if ((now - lastImuTick) >= kImuPeriod) {
            // ReadData() reads accel/gyro and magnetometer in one pass;
            // ReadMagData() returns the mag values cached during that read.
            latestImu = Hardware::imu_.ReadData();
            latestMag = Hardware::imu_.ReadMagData();
            const uint32_t sampleTimestampMs = CaptureTimestampMs();
            latestImu.timestampMs = sampleTimestampMs;
            latestMag.timestampMs = sampleTimestampMs;
            xQueueSend(SharedData::gI2C2IMUQueue, &latestImu, 0);
            xQueueSend(SharedData::gI2C2MagQueue, &latestMag, 0);
            UpdateSnapshot(&Common::SensorSnapshot::imuData, latestImu);
            UpdateSnapshot(&Common::SensorSnapshot::magData, latestMag);
            lastImuTick = now;
        }

        if ((now - lastVoltageTick) >= kVoltagePeriod) {
            latestVoltage = Hardware::voltageSensor_.ReadData();
            latestVoltage.timestampMs = CaptureTimestampMs();
            xQueueSend(SharedData::gI2C2VoltageQueue, &latestVoltage, 0);
            UpdateSnapshot(&Common::SensorSnapshot::voltageData, latestVoltage);
            lastVoltageTick = now;
        }

        // 5 ms loop granularity gives 200 Hz scheduling resolution.
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
