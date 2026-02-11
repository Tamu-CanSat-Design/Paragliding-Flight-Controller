#ifndef HARDWARE_EXAMPLE_IMU_DRIVER_H
#define HARDWARE_EXAMPLE_IMU_DRIVER_H

#include "interfaces/IMU.h"

// Example IMU driver skeleton. Use this as a template for new drivers.
class ExampleImu : public IIMU {
public:
    static constexpr const char* kSensorName = "Example IMU";

    const char* SensorName() const override { return kSensorName; }
    // Return true if initialization succeeds.
    bool Initialize() override;

    // Return the latest IMU reading.
    IMUData ReadData() override;

    // Optional calibration routine.
    void Calibrate() override;

    // Allows tests or simulation to inject values.
    void setIMUData(const IMUData& data) override;

private:
    // Cache the last known sample so ReadData is quick.
    IMUData data_{};
};

#endif  // HARDWARE_EXAMPLE_IMU_DRIVER_H
