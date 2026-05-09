#include "Hardware/drivers/Magnetometer.h"

#include <Arduino.h>
#include <Wire.h>

namespace {
// IIS2MDC register map + scaling constants.
constexpr uint8_t kMagAddress = 0x1E;
constexpr uint8_t kMagWhoAmIReg = 0x4F;
constexpr uint8_t kMagWhoAmIValue = 0x40;
constexpr uint8_t kMagCfgAReg = 0x60;
constexpr uint8_t kMagCfgBReg = 0x61;
constexpr uint8_t kMagCfgCReg = 0x62;
constexpr uint8_t kMagOutXLReg = 0x68;

constexpr uint8_t kMagCfgAReset = 0x20;
constexpr uint8_t kMagCfgA = 0x88;  // Temp comp + 50 Hz + continuous mode.
constexpr uint8_t kMagCfgB = 0x00;
constexpr uint8_t kMagCfgC = 0x20;  // BDU enabled.

constexpr float kMagMicroTeslaPerLsb = 0.15f;  // 1.5 mG/LSB.
}

bool Magnetometer::Initialize() {
    data_ = {};
    diagnostics_ = {};
    initialized_ = false;

    Wire1.begin();
    Wire1.setClock(Common::I2C_BAUDRATE);
    delay(10);
    device_.setAddress(kMagAddress);

    uint8_t whoami = 0;
    bool whoamiOk = device_.ReadRegister(kMagWhoAmIReg, &whoami);
    Serial.printf("WHO_AM_I read ok: %s, value: 0x%02X (expect 0x40)\n", whoamiOk ? "yes" : "no", whoami);
    if (!whoamiOk || whoami != kMagWhoAmIValue) {
        ++diagnostics_.initFailureCount;
        return false;
    }
    Serial.println("Magnetometer WHO_AM_I OK");
    
    if (!device_.WriteRegister(kMagCfgAReg, kMagCfgAReset)) {
        ++diagnostics_.initFailureCount;
        return false;
    }
    Serial.println("Magnetometer reset OK");
    delay(10);
    if (!device_.WriteRegister(kMagCfgAReg, kMagCfgA)) {
        ++diagnostics_.initFailureCount;
        return false;
    }
    Serial.println("Magnetometer CFG_A OK");
    if (!device_.WriteRegister(kMagCfgBReg, kMagCfgB)) {
        ++diagnostics_.initFailureCount;
        return false;
    }
    Serial.println("Magnetometer CFG_B OK");
    if (!device_.WriteRegister(kMagCfgCReg, kMagCfgC)) {
        ++diagnostics_.initFailureCount;
        return false;
    }
    Serial.println("Magnetometer CFG_C OK");
    initialized_ = true;
    return true;
}

Common::MagData Magnetometer::ReadData() {
    if (!initialized_) {
        ++diagnostics_.staleReturnCount;
        return data_;
    }

    uint8_t raw[6] = {};
    if (!device_.ReadRegisters(kMagOutXLReg, raw, sizeof(raw), false)) {
        ++diagnostics_.readFailureCount;
        ++diagnostics_.staleReturnCount;
        return data_;
    }

    const int16_t mx = static_cast<int16_t>((raw[1] << 8) | raw[0]);
    const int16_t my = static_cast<int16_t>((raw[3] << 8) | raw[2]);
    const int16_t mz = static_cast<int16_t>((raw[5] << 8) | raw[4]);

    data_ = Common::MagData{
        mx * kMagMicroTeslaPerLsb,
        my * kMagMicroTeslaPerLsb,
        mz * kMagMicroTeslaPerLsb,
    };
    return data_;
}

Magnetometer::Diagnostics Magnetometer::GetDiagnostics() const {
    return diagnostics_;
}

void Magnetometer::ClearDiagnostics() {
    diagnostics_ = {};
}
