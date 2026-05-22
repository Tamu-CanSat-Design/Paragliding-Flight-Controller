#include "Hardware/drivers/BarometerBMP3.h"

#include <Arduino.h>
#include <Wire.h>
#include <cmath>

namespace {
constexpr float kDefaultReferencePressureHpa = 1013.25f;
constexpr uint8_t kReviewedIirFilterSetting = BMP3_IIR_FILTER_COEFF_3;

BMP3_INTF_RET_TYPE Bmp3I2cRead(uint8_t regAddr, uint8_t* regData, uint32_t len, void* intfPtr) {
    if (regData == nullptr || intfPtr == nullptr || len == 0U || len > 255U) {
        return static_cast<BMP3_INTF_RET_TYPE>(-1);
    }

    const uint8_t address = *static_cast<uint8_t*>(intfPtr);
    Wire.beginTransmission(address);
    Wire.write(regAddr);
    const uint8_t txResult = Wire.endTransmission(false);
    if (txResult != 0U) {
        return static_cast<BMP3_INTF_RET_TYPE>(txResult);
    }

    const size_t received = Wire.requestFrom(address, static_cast<uint8_t>(len));
    if (received != len) {
        return static_cast<BMP3_INTF_RET_TYPE>(-2);
    }

    for (uint32_t i = 0; i < len; ++i) {
        const int value = Wire.read();
        if (value < 0 || value > 255) {
            return static_cast<BMP3_INTF_RET_TYPE>(-3);
        }
        regData[i] = static_cast<uint8_t>(value);
    }

    return BMP3_INTF_RET_SUCCESS;
}

BMP3_INTF_RET_TYPE Bmp3I2cWrite(uint8_t regAddr, const uint8_t* regData, uint32_t len, void* intfPtr) {
    if (regData == nullptr || intfPtr == nullptr || len == 0U || len > 255U) {
        return static_cast<BMP3_INTF_RET_TYPE>(-1);
    }

    const uint8_t address = *static_cast<uint8_t*>(intfPtr);
    Wire.beginTransmission(address);
    Wire.write(regAddr);
    for (uint32_t i = 0; i < len; ++i) {
        Wire.write(regData[i]);
    }

    const uint8_t txResult = Wire.endTransmission();
    return txResult == 0U ? BMP3_INTF_RET_SUCCESS : static_cast<BMP3_INTF_RET_TYPE>(txResult);
}

void Bmp3DelayUs(uint32_t period, void* intfPtr) {
    (void)intfPtr;
    if (period >= 1000U) {
        delay(period / 1000U);
    }
    const uint32_t remainder = period % 1000U;
    if (remainder > 0U) {
        delayMicroseconds(remainder);
    }
}

void SetDeviceCallbacks(bmp3_dev* device, uint8_t* address) {
    device->intf = BMP3_I2C_INTF;
    device->intf_ptr = address;
    device->read = Bmp3I2cRead;
    device->write = Bmp3I2cWrite;
    device->delay_us = Bmp3DelayUs;
}

float SensorPressurePa(const bmp3_data& sensorData) {
#ifdef BMP3_FLOAT_COMPENSATION
    return static_cast<float>(sensorData.pressure);
#else
    return static_cast<float>(sensorData.pressure) * 0.01f;
#endif
}

float SensorTemperatureC(const bmp3_data& sensorData) {
#ifdef BMP3_FLOAT_COMPENSATION
    return static_cast<float>(sensorData.temperature);
#else
    return static_cast<float>(sensorData.temperature) * 0.01f;
#endif
}
}  // namespace

bool BarometerBMP3::Initialize() {
    initialized_ = false;
    data_ = {};
    diagnostics_.foundAddress = 0;
    diagnostics_.chipId = 0;
    settings_ = {};
    referencePressureHpa_ = kDefaultReferencePressureHpa;

    Wire.begin();
    Wire.setClock(Common::I2C_BAUDRATE);

    if (!TryInitializeAtAddress(BMP3_ADDR_I2C_PRIM) && !TryInitializeAtAddress(BMP3_ADDR_I2C_SEC)) {
        Serial.printf("BarometerBMP3: init failed at 0x%02X and 0x%02X (api=%d intf=%d)\n",
                      BMP3_ADDR_I2C_PRIM,
                      BMP3_ADDR_I2C_SEC,
                      diagnostics_.lastInitApiResult,
                      diagnostics_.lastInitInterfaceResult);
        ++diagnostics_.initFailureCount;
        return false;
    }

    if (!ConfigureSensor()) {
        Serial.printf("BarometerBMP3: configure failed (api=%d intf=%d)\n",
                      diagnostics_.lastConfigApiResult,
                      diagnostics_.lastConfigInterfaceResult);
        ++diagnostics_.initFailureCount;
        return false;
    }

    initialized_ = true;
    Serial.printf("BarometerBMP3: initialized at 0x%02X chip=0x%02X\n",
                  diagnostics_.foundAddress,
                  diagnostics_.chipId);
    return true;
}

Common::BarometerData BarometerBMP3::ReadData() {
    if (!initialized_) {
        ++diagnostics_.staleReturnCount;
        return data_;
    }

    ++diagnostics_.totalReadCount;

    bmp3_status status = {};
    int8_t rslt = bmp3_get_status(&status, &device_);
    diagnostics_.lastReadApiResult = rslt;
    diagnostics_.lastReadInterfaceResult = device_.intf_rslt;
    if (rslt != BMP3_OK) {
        RecordReadFailure(rslt);
        return data_;
    }

    if (status.sensor.drdy_press == 0U || status.sensor.drdy_temp == 0U) {
        ++diagnostics_.notReadyCount;
        ++diagnostics_.staleReturnCount;
        return data_;
    }

    bmp3_data sensorData = {};
    rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &sensorData, &device_);
    diagnostics_.lastReadApiResult = rslt;
    diagnostics_.lastReadInterfaceResult = device_.intf_rslt;
    if (rslt != BMP3_OK) {
        RecordReadFailure(rslt);
        return data_;
    }

    const float pressurePa = SensorPressurePa(sensorData);
    const float temperatureC = SensorTemperatureC(sensorData);
    if (!std::isfinite(pressurePa) || !std::isfinite(temperatureC) || pressurePa <= 0.0f) {
        RecordReadFailure(BMP3_W_MIN_PRES);
        return data_;
    }

    diagnostics_.consecutiveReadFailures = 0;
    data_.temperature = temperatureC;
    data_.pressure = pressurePa * 0.01f;
    data_.altitude = PressureToRelativeAltitude(data_.pressure);
    return data_;
}

bool BarometerBMP3::SetCalibrationPressure(float pressureHpa) {
    if (!std::isfinite(pressureHpa) || pressureHpa <= 0.0f) {
        return false;
    }
    referencePressureHpa_ = pressureHpa;
    return true;
}

BarometerBMP3::Diagnostics BarometerBMP3::GetDiagnostics() const {
    return diagnostics_;
}

bool BarometerBMP3::TryInitializeAtAddress(uint8_t address) {
    i2cAddress_ = address;
    device_ = {};
    SetDeviceCallbacks(&device_, &i2cAddress_);

    const int8_t rslt = bmp3_init(&device_);
    if (address == BMP3_ADDR_I2C_PRIM) {
        diagnostics_.primaryInitApiResult = rslt;
        diagnostics_.primaryInitInterfaceResult = device_.intf_rslt;
        diagnostics_.primaryInitChipId = device_.chip_id;
    } else {
        diagnostics_.secondaryInitApiResult = rslt;
        diagnostics_.secondaryInitInterfaceResult = device_.intf_rslt;
        diagnostics_.secondaryInitChipId = device_.chip_id;
    }

    diagnostics_.lastInitApiResult = rslt;
    diagnostics_.lastInitInterfaceResult = device_.intf_rslt;
    diagnostics_.lastInitChipId = device_.chip_id;
    if (rslt != BMP3_OK || device_.chip_id != BMP390_CHIP_ID) {
        return false;
    }

    diagnostics_.foundAddress = address;
    diagnostics_.chipId = device_.chip_id;
    return true;
}

bool BarometerBMP3::ConfigureSensor() {
    settings_ = {};
    settings_.press_en = BMP3_ENABLE;
    settings_.temp_en = BMP3_ENABLE;
    settings_.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    settings_.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    settings_.odr_filter.odr = BMP3_ODR_50_HZ;
    settings_.odr_filter.iir_filter = kReviewedIirFilterSetting;

    const uint32_t desiredSettings = BMP3_SEL_PRESS_EN
                                   | BMP3_SEL_TEMP_EN
                                   | BMP3_SEL_PRESS_OS
                                   | BMP3_SEL_TEMP_OS
                                   | BMP3_SEL_IIR_FILTER
                                   | BMP3_SEL_ODR;

    int8_t rslt = bmp3_set_sensor_settings(desiredSettings, &settings_, &device_);
    diagnostics_.lastConfigApiResult = rslt;
    diagnostics_.lastConfigInterfaceResult = device_.intf_rslt;
    if (rslt != BMP3_OK) {
        return false;
    }

    settings_.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings_, &device_);
    diagnostics_.lastConfigApiResult = rslt;
    diagnostics_.lastConfigInterfaceResult = device_.intf_rslt;
    return rslt == BMP3_OK;
}

void BarometerBMP3::RecordReadFailure(int8_t apiResult) {
    diagnostics_.lastReadApiResult = apiResult;
    diagnostics_.lastReadInterfaceResult = device_.intf_rslt;
    ++diagnostics_.readFailureCount;
    ++diagnostics_.consecutiveReadFailures;
    if (diagnostics_.consecutiveReadFailures > diagnostics_.maxConsecutiveReadFailures) {
        diagnostics_.maxConsecutiveReadFailures = diagnostics_.consecutiveReadFailures;
    }
    ++diagnostics_.staleReturnCount;
}

float BarometerBMP3::PressureToRelativeAltitude(float pressureHpa) const {
    if (pressureHpa <= 0.0f || referencePressureHpa_ <= 0.0f) {
        return 0.0f;
    }
    const float ratio = pressureHpa / referencePressureHpa_;
    return 44330.0f * (1.0f - std::pow(ratio, 0.190294957f));
}
