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
    (void)pvParameters; // silence unused parameter warning

    Common::BarometerData latestBaro = {};
    Common::GPSData latestGps = {};
    Common::LidarData latestLidar = {};

    while(true) {
        HealthMonitor::Kick(HealthMonitor::TaskId::I2C1Bus);

        // Process pending control actions before publishing the next sensor sample.
        HandleI2C1ControlCommands();

        // Read barometer, GPS, and lidar data from I2C bus 1
        latestBaro = Hardware::barometer_.ReadData();
        // gHardwareManager.ReadGPS(&latestGps);
        // gHardwareManager.ReadLidar(&latestLidar);

        // Send the latest data to the navigation task via queues
        xQueueSend(SharedData::gI2C1BarometerQueue, &latestBaro, portMAX_DELAY);
        xQueueSend(SharedData::gI2C1GPSQueue, &latestGps, portMAX_DELAY);
        xQueueSend(SharedData::gI2C1LidarQueue, &latestLidar, portMAX_DELAY);

        // Sleep for a short period to yield to other tasks (e.g., navigation)
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz read rate
    }
}

void Tasks::I2C2BusTask(void* pvParameters) {
	(void)pvParameters;

	Common::IMUData latestImu = {};
	Common::MagData latestMag = {};
	Common::VoltageData latestVoltage = {};

	while (true) {
        HealthMonitor::Kick(HealthMonitor::TaskId::I2C2Bus);

		// TODO: Replace with actual hardware reads once the drivers are integrated.
		// gHardwareManager.ReadIMU(&latestImu);
		// gHardwareManager.ReadMagnetometer(&latestMag);
		// gHardwareManager.ReadVoltage(&latestVoltage);

		xQueueSend(SharedData::gI2C2IMUQueue, &latestImu, portMAX_DELAY);
		xQueueSend(SharedData::gI2C2MagQueue, &latestMag, portMAX_DELAY);
		xQueueSend(SharedData::gI2C2VoltageQueue, &latestVoltage, portMAX_DELAY);

		vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz update cadence
	}
}
