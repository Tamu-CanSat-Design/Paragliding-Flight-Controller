#ifndef HARDWARE_LIDAR_DRIVER_H
#define HARDWARE_LIDAR_DRIVER_H

#include <cstdint>

#include "../bus/I2CDevice.h"
#include "../interfaces/LidarInterface.h"

class Lidar : public ILidar {
public:
    enum class MeasurementStatus : uint8_t {
        kBusy = 0,
        kReady = 1,
        kError = 2,
    };

    static constexpr const char* kSensorName = "Garmin LIDAR-Lite v4 LED";

    explicit Lidar(TwoWire& wire);
    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::LidarData ReadData() override;
    bool StartMeasurement(bool withBiasCorrection = true);
    MeasurementStatus ReadMeasurementIfReady(Common::LidarData* data);

private:
    // Talks to the sensor over I2C register transactions.
    I2CDevice device_;
    // Last successfully measured value so callers always get a stable sample.
    Common::LidarData data_{};
    // Prevents bus access before startup/config are complete.
    bool initialized_ = false;
    bool measurementInProgress_ = false;

    // Reads STATUS (0x01), which includes the BUSY bit and error flags.
    bool ReadStatus(uint8_t* status) const;
    // Starts a new ranging cycle by writing ACQ_COMMANDS (0x00).
    bool TriggerMeasurement(bool withBiasCorrection);
    // Polls STATUS.BUSY until the conversion is complete or timeout expires.
    bool WaitForMeasurementComplete(uint32_t timeoutMs);
    // Reads FULL_DELAY_LOW/HIGH (0x10/0x11) and returns distance in centimeters.
    bool ReadDistanceCentimeters(uint16_t* distanceCm);
};

#endif  // HARDWARE_LIDAR_DRIVER_H
