#include "TaskHandler.h"




void Tasks::I2C1BusTask(void* pvParameters) {
    (void)pvParameters; // silence unused parameter warning

    Common::BarometerData latestBaro = {};
    Common::GPSData latestGps = {};
    Common::LidarData latestLidar = {};

    while(true) {
        // Read barometer, GPS, and lidar data from I2C bus 1
        // gHardwareManager.ReadBarometer(&latestBaro);
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