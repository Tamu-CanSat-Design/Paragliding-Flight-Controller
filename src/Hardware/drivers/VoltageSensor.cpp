#include "Hardware/drivers/VoltageSensor.h"

#include <Arduino.h>
#include <Wire.h>

namespace {
// INA260 register map (datasheet section 8.6).
constexpr uint8_t kRegConfiguration = 0x00;
constexpr uint8_t kRegCurrent = 0x01;
constexpr uint8_t kRegManufacturerId = 0xFE;
constexpr uint8_t kRegDieId = 0xFF;

// A0/A1 pin strap combinations produce 16 contiguous 7-bit addresses: 0x40 to 0x4F.
constexpr uint8_t kAddressMin = 0x40;
constexpr uint8_t kAddressMax = 0x4F;

// Identification values from datasheet register reset values.
constexpr uint16_t kManufacturerIdTi = 0x5449;
constexpr uint16_t kDieIdMask = 0xFFF0;          // Ignore revision nibble.
constexpr uint16_t kDieIdExpectedMasked = 0x2270;

// Configuration values.
constexpr uint16_t kConfigReset = 0x8000;        // RST bit, self-clearing.
constexpr uint16_t kConfigDefault = 0x6127;      // Datasheet power-on default.

// Measurement scaling factors from datasheet register descriptions.
constexpr float kCurrentLsbAmps = 0.00125f;      // 1.25 mA / LSB.
constexpr float kBusVoltageLsbVolts = 0.00125f;  // 1.25 mV / LSB.
constexpr float kPowerLsbWatts = 0.01f;          // 10 mW / LSB.
}  // namespace

bool VoltageSensor::Initialize() {
    // Clear cached sample and state before touching hardware.
    data_ = {};
    initialized_ = false;

    // Initialize I2C bus at the project-wide configured bitrate.
    Wire.begin();
    Wire.setClock(Common::I2C_BAUDRATE);

    // Find the actual INA260 address from A0/A1 strap configuration.
    if (!ProbeAndSelectAddress()) {
        return false;
    }

    // Reset and program a known configuration so measurements are deterministic.
    if (!Configure()) {
        return false;
    }

    initialized_ = true;
    return true;
}

Common::VoltageData VoltageSensor::ReadData() {
    // Keep returning last good sample until hardware is initialized.
    if (!initialized_) {
        return data_;
    }

    // Read current, bus voltage, and power in one burst from register 0x01..0x03.
    uint8_t raw[6] = {};
    if (!device_.ReadRegisters(kRegCurrent, raw, sizeof(raw), false)) {
        return data_;
    }

    // INA260 transmits register words MSB first.
    const int16_t rawCurrent = static_cast<int16_t>(
        (static_cast<uint16_t>(raw[0]) << 8) | static_cast<uint16_t>(raw[1]));
    const uint16_t rawVoltage = static_cast<uint16_t>(
        (static_cast<uint16_t>(raw[2]) << 8) | static_cast<uint16_t>(raw[3]));
    const uint16_t rawPower = static_cast<uint16_t>(
        (static_cast<uint16_t>(raw[4]) << 8) | static_cast<uint16_t>(raw[5]));

    // Convert fixed-point register values into engineering units.
    Common::VoltageData sample{};
    sample.current = static_cast<float>(rawCurrent) * kCurrentLsbAmps;
    sample.voltage = static_cast<float>(rawVoltage) * kBusVoltageLsbVolts;
    sample.power = static_cast<float>(rawPower) * kPowerLsbWatts;

    data_ = sample;
    return data_;
}

bool VoltageSensor::ProbeAndSelectAddress() {
    // Scan all valid INA260 addresses and validate using manufacturer + die ID.
    for (uint8_t address = kAddressMin; address <= kAddressMax; ++address) {
        device_.setAddress(address);

        uint16_t manufacturerId = 0;
        if (!ReadRegister16(kRegManufacturerId, &manufacturerId)) {
            continue;
        }
        if (manufacturerId != kManufacturerIdTi) {
            continue;
        }

        uint16_t dieId = 0;
        if (!ReadRegister16(kRegDieId, &dieId)) {
            continue;
        }
        if ((dieId & kDieIdMask) != kDieIdExpectedMasked) {
            continue;
        }

        // Matching IDs found; keep this address selected.
        return true;
    }

    return false;
}

bool VoltageSensor::Configure() {
    // Trigger a software reset to guarantee known defaults.
    if (!WriteRegister16(kRegConfiguration, kConfigReset)) {
        return false;
    }

    // Small delay lets reset complete before writing final configuration.
    delay(2);

    // Program documented default continuous measurement configuration.
    return WriteRegister16(kRegConfiguration, kConfigDefault);
}

bool VoltageSensor::WriteRegister16(uint8_t reg, uint16_t value) const {
    // INA260 expects register writes as pointer + MSB + LSB.
    Wire.beginTransmission(device_.address());
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>((value >> 8) & 0xFF));
    Wire.write(static_cast<uint8_t>(value & 0xFF));
    return Wire.endTransmission() == 0;
}

bool VoltageSensor::ReadRegister16(uint8_t reg, uint16_t* value) const {
    if (value == nullptr) {
        return false;
    }

    uint8_t raw[2] = {};
    if (!device_.ReadRegisters(reg, raw, sizeof(raw), false)) {
        return false;
    }

    *value = static_cast<uint16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);
    return true;
}
