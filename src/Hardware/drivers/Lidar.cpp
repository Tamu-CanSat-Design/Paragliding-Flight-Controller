#include "Hardware/drivers/Lidar.h"

#include <Arduino.h>
#include <Wire.h>

namespace {
// Datasheet: default 7-bit I2C address is 0x62.
constexpr uint8_t kI2cAddress = 0x62;

// Datasheet register map used by the basic ranging workflow.
constexpr uint8_t kRegAcqCommands = 0x00;
constexpr uint8_t kRegStatus = 0x01;
constexpr uint8_t kRegAcquisitionCount = 0x05;
constexpr uint8_t kRegDistanceLow = 0x10;
constexpr uint8_t kRegDeviceVersion = 0xE1;
constexpr uint8_t kRegPowerMode = 0xE2;
constexpr uint8_t kRegQuickTermination = 0xE5;
constexpr uint8_t kRegHighAccuracyMode = 0xEB;

// ACQ_COMMANDS values from the datasheet.
constexpr uint8_t kCommandDistanceNoBiasCorrection = 0x03;
constexpr uint8_t kCommandDistanceWithBiasCorrection = 0x04;

// STATUS register bits from the datasheet.
constexpr uint8_t kStatusBitBusy = 1 << 0;
constexpr uint8_t kStatusBitDcError = 1 << 5;

// Datasheet notes startup time is roughly 22 ms after power-up/reset.
constexpr uint32_t kStartupDelayMs = 25;
// Typical readings are fast; timeout keeps the task from hanging on bus faults.
constexpr uint32_t kMeasurementTimeoutMs = 40;

// These values match documented defaults but are written explicitly so device state
// is deterministic even if NVM settings were changed in a previous session.
constexpr uint8_t kAcquisitionCountDefault = 0xFF;
constexpr uint8_t kPowerModeAlwaysOn = 0xFF;
constexpr uint8_t kQuickTerminationDisabled = 0x08;
constexpr uint8_t kHighAccuracyDisabled = 0x00;
constexpr uint8_t kHighAccuracyDefault = 0x14;

// Converts the sensor's integer centimeters output into SI meters.
constexpr float kCentimetersToMeters = 0.01f;
}  // namespace

Lidar::Lidar(TwoWire& wire)
    : device_(0, wire) {}

bool Lidar::Initialize() {
    // Clear state first so failed init leaves the object in a known-safe condition.
    data_ = {};
    initialized_ = false;

    // Bring up I2C using the shared project settings.
    device_.wire().begin();
    device_.wire().setClock(Common::I2C_BAUDRATE);

    // Configure the bus wrapper for the LIDAR-Lite default address.
    device_.setAddress(kI2cAddress);

    // Allow the device to finish its internal boot and self-test sequence.
    delay(kStartupDelayMs);

    // A single register read confirms the sensor responds at this address.
    uint8_t version = 0;
    if (!device_.ReadRegister(kRegDeviceVersion, &version)) {
        return false;
    }

    // Explicitly apply core ranging defaults so behavior is predictable.
    // Datasheet note for POWER_MODE: disable high-accuracy mode before adjusting power mode.
    if (!device_.WriteRegister(kRegHighAccuracyMode, kHighAccuracyDisabled)) {
        return false;
    }
    if (!device_.WriteRegister(kRegPowerMode, kPowerModeAlwaysOn)) {
        return false;
    }
    if (!device_.WriteRegister(kRegAcquisitionCount, kAcquisitionCountDefault)) {
        return false;
    }
    if (!device_.WriteRegister(kRegQuickTermination, kQuickTerminationDisabled)) {
        return false;
    }
    if (!device_.WriteRegister(kRegHighAccuracyMode, kHighAccuracyDefault)) {
        return false;
    }

    initialized_ = true;
    return true;
}

Common::LidarData Lidar::ReadData() {
    // Return the last good sample if initialization has not completed.
    if (!initialized_) {
        return data_;
    }

    // Start a new range measurement with receiver bias correction enabled.
    // Datasheet "Obtaining Measurements from the I2C Interface", step 1.
    if (!TriggerMeasurement(true)) {
        return data_;
    }

    // Poll STATUS.BUSY until the conversion is complete or timeout is hit.
    if (!WaitForMeasurementComplete(kMeasurementTimeoutMs)) {
        return data_;
    }

    // Read the 16-bit distance result in centimeters (LOW then HIGH byte).
    uint16_t distanceCm = 0;
    if (!ReadDistanceCentimeters(&distanceCm)) {
        return data_;
    }

    // Datasheet notes that invalid signal conditions can return 1 cm.
    // Ignore this sentinel and keep the previous valid sample for flight logic.
    if (distanceCm <= 1) {
        return data_;
    }

    data_.distanceMeters = static_cast<float>(distanceCm) * kCentimetersToMeters;
    return data_;
}

bool Lidar::ReadStatus(uint8_t* status) const {
    if (status == nullptr) {
        return false;
    }
    return device_.ReadRegister(kRegStatus, status);
}

bool Lidar::TriggerMeasurement(bool withBiasCorrection) {
    // Datasheet command 0x04 uses bias correction, 0x03 skips it for speed.
    const uint8_t command = withBiasCorrection ? kCommandDistanceWithBiasCorrection
                                               : kCommandDistanceNoBiasCorrection;
    return device_.WriteRegister(kRegAcqCommands, command);
}

bool Lidar::WaitForMeasurementComplete(uint32_t timeoutMs) {
    const uint32_t startMs = millis();
    while ((millis() - startMs) < timeoutMs) {
        uint8_t status = 0;
        if (!ReadStatus(&status)) {
            return false;
        }

        // Stop waiting as soon as BUSY clears.
        if ((status & kStatusBitBusy) == 0) {
            // DC correction failure means the returned distance is expected to be inaccurate.
            return (status & kStatusBitDcError) == 0;
        }

        // Short sleep limits bus traffic while keeping latency low.
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Timed out waiting for BUSY to clear.
    return false;
}

bool Lidar::ReadDistanceCentimeters(uint16_t* distanceCm) {
    if (distanceCm == nullptr) {
        return false;
    }

    uint8_t rawDistance[2] = {};
    // Device auto-increments register pointer in block reads per datasheet.
    if (!device_.ReadRegisters(kRegDistanceLow, rawDistance, sizeof(rawDistance), false)) {
        return false;
    }

    *distanceCm = static_cast<uint16_t>(rawDistance[0])
        | (static_cast<uint16_t>(rawDistance[1]) << 8);
    return true;
}
