#include "TaskHandler.h"

namespace {
// The I2C1 task owns the barometer, so maintenance actions such as rezeroing
// are routed here from mission logic instead of touching the driver cross-task.
void HandleI2C1ControlCommands() {
    Common::I2C1ControlCommand command{};
    while (xQueueReceive(SharedData::gI2C1ControlQueue, &command, 0) == pdTRUE) {
        Common::I2C1ControlResult result{};
        result.type = command.type;

        switch (command.type) {
            case Common::I2C1ControlCommandType::kRezeroBarometer:
                result.success = Hardware::barometer_.RezeroReferencePressure();
                break;
            default:
                result.success = false;
                break;
        }

        xQueueSend(SharedData::gI2C1ControlResultQueue, &result, portMAX_DELAY);
    }
}
}


void Tasks::I2C1BusTask(void* pvParameters) {
    (void)pvParameters;

    Common::BarometerData latestBaro  = {};
    Common::GPSData       latestGps   = {};
    Common::LidarData     latestLidar = {};

    // Target rates: barometer 50 Hz, lidar 20 Hz, GPS 5 Hz.
    constexpr TickType_t kBaroPeriod  = pdMS_TO_TICKS(20);  // 50 Hz — matches BMP390 ODR
    constexpr TickType_t kLidarPeriod = pdMS_TO_TICKS(50);  // 20 Hz
    constexpr TickType_t kGpsPeriod   = pdMS_TO_TICKS(200); // 5 Hz

    TickType_t lastBaroTick  = 0;
    TickType_t lastLidarTick = 0;
    TickType_t lastGpsTick   = 0;

    while (true) {
        HealthMonitor::Kick(HealthMonitor::TaskId::I2C1Bus);

        // Process pending control actions before publishing the next sensor sample.
        HandleI2C1ControlCommands();

        const TickType_t now = xTaskGetTickCount();

        if ((now - lastBaroTick) >= kBaroPeriod) {
            latestBaro = Hardware::barometer_.ReadData();
            xQueueSend(SharedData::gI2C1BarometerQueue, &latestBaro, 0);
            lastBaroTick = now;
        }

        if ((now - lastLidarTick) >= kLidarPeriod) {
            latestLidar = Hardware::lidar_.ReadData();
            xQueueSend(SharedData::gI2C1LidarQueue, &latestLidar, 0);
            lastLidarTick = now;
        }

        if ((now - lastGpsTick) >= kGpsPeriod) {
            latestGps = Hardware::gps_.ReadData();
            xQueueSend(SharedData::gI2C1GPSQueue, &latestGps, 0);
            lastGpsTick = now;
        }

        // 5 ms loop granularity gives 200 Hz scheduling resolution, which is
        // tight enough to hit all three target sensor rates accurately.
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void Tasks::I2C2BusTask(void* pvParameters) {
    (void)pvParameters;

    Common::IMUData     latestImu     = {};
    Common::MagData     latestMag     = {};
    Common::VoltageData latestVoltage = {};

    // Target rates: IMU+mag 104 Hz (10 ms is the closest integer-ms period),
    // voltage 7 Hz (143 ms).
    constexpr TickType_t kImuPeriod     = pdMS_TO_TICKS(10);  // ~100 Hz (LSM6DSR ODR is 104 Hz)
    constexpr TickType_t kVoltagePeriod = pdMS_TO_TICKS(143); // ~7 Hz

    TickType_t lastImuTick     = 0;
    TickType_t lastVoltageTick = 0;

    while (true) {
        HealthMonitor::Kick(HealthMonitor::TaskId::I2C2Bus);

        const TickType_t now = xTaskGetTickCount();

        if ((now - lastImuTick) >= kImuPeriod) {
            // ReadData() reads accel/gyro and magnetometer in one pass;
            // ReadMagData() returns the mag values cached during that read.
            latestImu = Hardware::imu_.ReadData();
            latestMag = Hardware::imu_.ReadMagData();
            xQueueSend(SharedData::gI2C2IMUQueue, &latestImu, 0);
            xQueueSend(SharedData::gI2C2MagQueue, &latestMag, 0);
            lastImuTick = now;
        }

        if ((now - lastVoltageTick) >= kVoltagePeriod) {
            latestVoltage = Hardware::voltageSensor_.ReadData();
            xQueueSend(SharedData::gI2C2VoltageQueue, &latestVoltage, 0);
            lastVoltageTick = now;
        }

        // 5 ms loop granularity gives 200 Hz scheduling resolution.
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
