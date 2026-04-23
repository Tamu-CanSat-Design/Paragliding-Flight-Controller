#include "Hardware/drivers/VoltageSensor.h"

#include <Arduino.h>
#include <Wire.h>

namespace {
// TODO(schematic): confirm I2C bus (Wire vs Wire1) and A0/A1 strapping.
// Default address 0x40 assumes A0=A1=GND.
constexpr uint8_t kIna260Address = 0x40;

// Register map (INA260, TI SBOS656C).
constexpr uint8_t kCurrentReg    = 0x01;
constexpr uint8_t kVoltageReg    = 0x02;
constexpr uint8_t kPowerReg      = 0x03;
constexpr uint8_t kManufIdReg    = 0xFE;

constexpr uint16_t kManufIdValue = 0x5449;  // Expected Manufacturer ID.

// INA260 uses big-endian byte order (MSB first) — opposite of IIS2MDC/ISM330DHCX.
// Byte combine: (buf[0] << 8) | buf[1]
constexpr float kCurrentLsbMa = 1.25f;  // 1.25 mA / LSB
constexpr float kVoltageLsbMv = 1.25f;  // 1.25 mV / LSB
constexpr float kPowerLsbMw   = 10.0f;  // 10 mW / LSB
}  // namespace

bool VoltageSensor::Initialize() {
    data_ = {};
    diagnostics_ = {};
    initialized_ = false;

    Wire.begin();
    Wire.setClock(Common::I2C_BAUDRATE);
    device_.setAddress(kIna260Address);

    // Verify device identity via big-endian 2-byte Manufacturer ID register.
    uint8_t id[2] = {};
    if (!device_.ReadRegisters(kManufIdReg, id, sizeof(id), false)) {
        ++diagnostics_.initFailureCount;
        return false;
    }
    const uint16_t manufId = static_cast<uint16_t>((static_cast<uint16_t>(id[0]) << 8) | id[1]);
    if (manufId != kManufIdValue) {
        ++diagnostics_.initFailureCount;
        return false;
    }

    initialized_ = true;
    return true;
}

Common::VoltageData VoltageSensor::ReadData() {
    if (!initialized_) {
        ++diagnostics_.staleReturnCount;
        return data_;
    }

    uint8_t raw[2] = {};

    if (!device_.ReadRegisters(kCurrentReg, raw, sizeof(raw), false)) {
        ++diagnostics_.readFailureCount;
        ++diagnostics_.staleReturnCount;
        return data_;
    }
    const int16_t currentRaw = static_cast<int16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);

    if (!device_.ReadRegisters(kVoltageReg, raw, sizeof(raw), false)) {
        ++diagnostics_.readFailureCount;
        ++diagnostics_.staleReturnCount;
        return data_;
    }
    const uint16_t voltageRaw = static_cast<uint16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);

    if (!device_.ReadRegisters(kPowerReg, raw, sizeof(raw), false)) {
        ++diagnostics_.readFailureCount;
        ++diagnostics_.staleReturnCount;
        return data_;
    }
    const uint16_t powerRaw = static_cast<uint16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);

    data_ = Common::VoltageData{
        voltageRaw * kVoltageLsbMv,
        currentRaw * kCurrentLsbMa,
        powerRaw   * kPowerLsbMw,
    };
    return data_;
}

VoltageSensor::Diagnostics VoltageSensor::GetDiagnostics() const {
    return diagnostics_;
}

void VoltageSensor::ClearDiagnostics() {
    diagnostics_ = {};
}
