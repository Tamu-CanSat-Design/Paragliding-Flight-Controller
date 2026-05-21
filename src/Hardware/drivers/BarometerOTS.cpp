#include "Hardware/drivers/BarometerOTS.h"

#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include <cstdio>

namespace {

// BMP390_ADDRESS_ADO_LOW/HIGH are already left-shifted (8-bit form).
// Wire takes 7-bit addresses, so right-shift when calling Wire functions.

uint8_t iic_init(void) {
    Wire.begin();
    Wire.setClock(400000);
    return 0;
}

uint8_t iic_deinit(void) {
    return 0;
}

uint8_t iic_read(uint8_t addr, uint8_t reg, uint8_t* buf, uint16_t len) {
    const uint8_t addr7 = static_cast<uint8_t>(addr >> 1);
    Wire.beginTransmission(addr7);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return 1;
    }
    const uint8_t received = Wire.requestFrom(addr7, static_cast<uint8_t>(len));
    if (received != len) {
        return 1;
    }
    for (uint16_t i = 0; i < len; ++i) {
        buf[i] = static_cast<uint8_t>(Wire.read());
    }
    return 0;
}

uint8_t iic_write(uint8_t addr, uint8_t reg, uint8_t* buf, uint16_t len) {
    const uint8_t addr7 = static_cast<uint8_t>(addr >> 1);
    Wire.beginTransmission(addr7);
    Wire.write(reg);
    for (uint16_t i = 0; i < len; ++i) {
        Wire.write(buf[i]);
    }
    return Wire.endTransmission() == 0 ? 0 : 1;
}

void delay_ms(uint32_t ms) {
    delay(ms);
}

void debug_print(const char* const fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
}

void SetHandleCallbacks(bmp390_handle_t* h) {
    h->iic_init    = iic_init;
    h->iic_deinit  = iic_deinit;
    h->iic_read    = iic_read;
    h->iic_write   = iic_write;
    h->delay_ms    = delay_ms;
    h->debug_print = debug_print;
}

}  // namespace

bool BarometerOTS::Initialize() {
    initialized_ = false;
    data_ = {};
    diagnostics_ = {};
    referencePressureHpa_ = 1013.25f;
    handle_ = {};

    SetHandleCallbacks(&handle_);
    handle_.iic_spi = BMP390_INTERFACE_IIC;

    // Probe low address first, fall back to high.
    handle_.iic_addr = BMP390_ADDRESS_ADO_LOW;
    uint8_t err = bmp390_init(&handle_);
    if (err != 0) {
        handle_.iic_addr = BMP390_ADDRESS_ADO_HIGH;
        err = bmp390_init(&handle_);
        if (err != 0) {
            Serial.printf("BarometerOTS: init failed on both addresses (err=%u)\n", err);
            diagnostics_.lastInitErrorCode = err;
            ++diagnostics_.initFailureCount;
            return false;
        }
    }
    diagnostics_.foundAddress = static_cast<uint8_t>(handle_.iic_addr >> 1);

    if ((err = bmp390_set_pressure_oversampling(&handle_, BMP390_OVERSAMPLING_x8)) != 0) {
        Serial.printf("BarometerOTS: set pressure oversampling failed (err=%u)\n", err);
        diagnostics_.lastInitErrorCode = err;
        ++diagnostics_.initFailureCount;
        bmp390_deinit(&handle_);
        return false;
    }
    if ((err = bmp390_set_temperature_oversampling(&handle_, BMP390_OVERSAMPLING_x2)) != 0) {
        Serial.printf("BarometerOTS: set temperature oversampling failed (err=%u)\n", err);
        diagnostics_.lastInitErrorCode = err;
        ++diagnostics_.initFailureCount;
        bmp390_deinit(&handle_);
        return false;
    }
    if ((err = bmp390_set_odr(&handle_, BMP390_ODR_50_HZ)) != 0) {
        Serial.printf("BarometerOTS: set ODR failed (err=%u)\n", err);
        diagnostics_.lastInitErrorCode = err;
        ++diagnostics_.initFailureCount;
        bmp390_deinit(&handle_);
        return false;
    }
    if ((err = bmp390_set_filter_coefficient(&handle_, BMP390_FILTER_COEFFICIENT_3)) != 0) {
        Serial.printf("BarometerOTS: set filter failed (err=%u)\n", err);
        diagnostics_.lastInitErrorCode = err;
        ++diagnostics_.initFailureCount;
        bmp390_deinit(&handle_);
        return false;
    }
    if ((err = bmp390_set_mode(&handle_, BMP390_MODE_NORMAL_MODE)) != 0) {
        Serial.printf("BarometerOTS: set mode failed (err=%u)\n", err);
        diagnostics_.lastInitErrorCode = err;
        ++diagnostics_.initFailureCount;
        bmp390_deinit(&handle_);
        return false;
    }

    initialized_ = true;
    Serial.printf("BarometerOTS: initialized at 0x%02X\n", diagnostics_.foundAddress);
    return true;
}

Common::BarometerData BarometerOTS::ReadData() {
    if (!initialized_) {
        ++diagnostics_.staleReturnCount;
        return data_;
    }

    ++diagnostics_.totalReadCount;

    uint32_t raw_temp = 0;
    float temp_c = 0.0f;
    uint32_t raw_press = 0;
    float press_pa = 0.0f;

    const uint8_t err = static_cast<uint8_t>(
        bmp390_read_temperature_pressure(&handle_, &raw_temp, &temp_c, &raw_press, &press_pa));
    if (err != 0) {
        diagnostics_.lastReadErrorCode = err;
        ++diagnostics_.readFailureCount;
        ++diagnostics_.consecutiveReadFailures;
        if (diagnostics_.consecutiveReadFailures > diagnostics_.maxConsecutiveReadFailures) {
            diagnostics_.maxConsecutiveReadFailures = diagnostics_.consecutiveReadFailures;
        }
        ++diagnostics_.staleReturnCount;
        return data_;
    }

    diagnostics_.consecutiveReadFailures = 0;
    data_.temperature = temp_c;
    data_.pressure    = press_pa * 0.01f;  // Pa -> hPa
    data_.altitude    = PressureToRelativeAltitude(data_.pressure);
    return data_;
}

bool BarometerOTS::SetCalibrationPressure(float pressureHpa) {
    if (!std::isfinite(pressureHpa) || pressureHpa <= 0.0f) {
        return false;
    }
    referencePressureHpa_ = pressureHpa;
    return true;
}

BarometerOTS::Diagnostics BarometerOTS::GetDiagnostics() const {
    return diagnostics_;
}

float BarometerOTS::PressureToRelativeAltitude(float pressureHpa) const {
    if (pressureHpa <= 0.0f || referencePressureHpa_ <= 0.0f) {
        return 0.0f;
    }
    const float ratio = pressureHpa / referencePressureHpa_;
    return 44330.0f * (1.0f - std::pow(ratio, 0.190294957f));
}
