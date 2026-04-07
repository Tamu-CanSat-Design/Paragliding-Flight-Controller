#include "Hardware/drivers/Barometer.h"



// Arduino core for delay and wire setup.
#include <Arduino.h>
// I2C transport.
#include <Wire.h>
// Math helpers for scaling and altitude.
#include <cmath>

namespace {
// Datasheet p40: "The 7-bit device address is 111011x."
// Two possible I2C addresses based on SDO pin so we can probe whichever is wired.
constexpr uint8_t kI2cAddressLow = 0x76;
constexpr uint8_t kI2cAddressHigh = 0x77;

// Datasheet p31: "Register 0x00 \"CHIP_ID\"".
// ID register used to confirm the device is really a BMP390.
constexpr uint8_t kRegChipId = 0x00;
// Datasheet p32: "Register 0x03 \"STATUS\"".set/config.
constexpr uint8_t kRegStatus = 0x03;
// Status register used to avoid issuing commands mid-re
// Datasheet p32: "Register 0x04 .. 0x06 Pressure Data".
// First data register of the pressure/temperature burst read to keep samples coherent.
constexpr uint8_t kRegDataStart = 0x04;
// Datasheet p36: "Register 0x1A \"IF_CONF\"".
// Interface configuration register; we keep defaults for stable I2C behavior.
constexpr uint8_t kRegIfConf = 0x1A;
// Datasheet p36: "Register 0x1B \"PWR_CTRL\"".
// Power control for enabling measurements and selecting normal/forced mode.
constexpr uint8_t kRegPwrCtrl = 0x1B;
// Datasheet p37: "Register 0x1C \"OSR\"".
// Oversampling configuration register to trade noise vs. bandwidth.
constexpr uint8_t kRegOsr = 0x1C;
// Datasheet p37: "Register 0x1D \"ODR\"".
// Output data rate configuration register to set the sampling cadence.
constexpr uint8_t kRegOdr = 0x1D;
// Datasheet p39: "Register 0x1F \"CONFIG\"".
// IIR filter configuration register to smooth noise before output.
constexpr uint8_t kRegConfig = 0x1F;
// Datasheet p28: "Memory Map Trimming Coefficients".
// Calibration block start address so we can read all trim coefficients in one burst.
constexpr uint8_t kRegCalibStart = 0x31;
// Datasheet p39: "Register 0x7E \"CMD\"".
// Command register used for soft reset to ensure known default state.
constexpr uint8_t kRegCmd = 0x7E;

// Expected chip ID for BMP390 to validate the probe.
constexpr uint8_t kChipId = 0x60;
// Command value for soft reset.
constexpr uint8_t kCmdSoftReset = 0xB6;

// STATUS bit indicating command decoder ready so we don't race configuration.
constexpr uint8_t kStatusCmdReady = 1 << 4;

// Oversampling settings for pressure: improve resolution at the cost of latency.
constexpr uint8_t kOsrPress = 0x03;  // x8 oversampling.
// Oversampling settings for temperature: adequate for compensation without extra delay.
constexpr uint8_t kOsrTemp = 0x01;   // x2 oversampling.
// Output data rate setting: 25 Hz is a balanced rate for flight telemetry.
constexpr uint8_t kOdr = 0x03;       // 25 Hz output data rate.
// IIR filter coefficient selection: modest smoothing without too much lag.
constexpr uint8_t kIirFilter = 0x02; // Coefficient 3.

// Power control: enable press+temp, normal mode for continuous measurements.
constexpr uint8_t kPwrCtrl = 0x33;   // Press + temp enable, normal mode.

// Capture a launch baseline only after pressure settles for a full window.
constexpr uint32_t kReferenceSamplePeriodMs = 100;
constexpr uint32_t kReferenceStableWindowMs = 5000;
constexpr uint32_t kReferenceCaptureTimeoutMs = 15000;
constexpr float kReferenceStableSpanHpa = 0.12f;
constexpr size_t kReferenceWindowSamples = kReferenceStableWindowMs / kReferenceSamplePeriodMs;

// Read an unsigned 16-bit little-endian value from a byte buffer for coefficient unpacking.
uint16_t ReadU16(const uint8_t* data, size_t index) {
    return static_cast<uint16_t>(data[index])
        | (static_cast<uint16_t>(data[index + 1]) << 8);
}

// Read a signed 16-bit little-endian value from a byte buffer for signed coefficients.
int16_t ReadS16(const uint8_t* data, size_t index) {
    return static_cast<int16_t>(ReadU16(data, index));
}
}  // namespace

Barometer::Barometer(TwoWire& wire)
    : device_(0, wire) {}

// Initialize the sensor: bus, probe, reset, load calibration, configure for reliable readings.
bool Barometer::Initialize() {
    // Reset state before touching hardware so stale values don't leak.
    initialized_ = false;
    // Clear last data sample so ReadData returns known defaults if needed.
    data_ = {};
    // Clear calibration cache so old trims are not reused.
    calib_ = {};
    referencePressureHpa_ = 0.0f;
    lastReadSucceeded_ = false;
    referenceCaptureInProgress_ = false;
    referenceCaptureHasSample_ = false;
    referenceCaptureStartMs_ = 0;
    lastReferenceSampleMs_ = 0;
    referenceSampleCount_ = 0;
    nextReferenceSampleIndex_ = 0;
    referenceWindowPressures_.fill(0.0f);

    // Start I2C bus so the device is addressable.
    device_.wire().begin();
    // Set bus speed to project-defined rate for consistent timing across sensors.
    device_.wire().setClock(Common::I2C_BAUDRATE);

    // Probe device at both possible I2C addresses because SDO may be tied high/low.
    uint8_t chip_id = 0;
    device_.setAddress(kI2cAddressLow);
    // Datasheet p31: "The \"CHIP_ID\" register contains the chip identification code."
    if (!device_.ReadRegister(kRegChipId, &chip_id) || chip_id != kChipId) {
        device_.setAddress(kI2cAddressHigh);
        if (!device_.ReadRegister(kRegChipId, &chip_id) || chip_id != kChipId) {
            return false;
        }
    }

    // Soft reset to known default state so configuration writes behave predictably.
    if (!SoftReset()) {
        return false;
    }

    // Read calibration coefficients from NVM because compensation depends on them.
    if (!ReadCalibration()) {
        return false;
    }

    // Configure oversampling, ODR, filter, and mode to balance noise and latency.
    if (!Configure()) {
        return false;
    }

    initialized_ = true;
    StartReferencePressureCapture(millis());
    // Baseline capture defines the startup point as zero altitude.
    data_.altitude = 0.0f;
    return true;
}

// Read the latest barometer sample and return it so callers get one coherent snapshot.
Common::BarometerData Barometer::ReadData() {
    // If not initialized, return last cached data to avoid bogus zeros.
    if (!initialized_) {
        return data_;
    }

    // Pull raw 24-bit pressure and temperature since compensation is required.
    uint32_t raw_press = 0;
    uint32_t raw_temp = 0;
    if (!ReadRaw(&raw_press, &raw_temp)) {
        lastReadSucceeded_ = false;
        return data_;
    }

    // Convert raw data using calibration formulas for physical units.
    Common::BarometerData data{};
    // Temperature compensation also sets t_lin used for pressure.
    data.temperature = CompensateTemperature(raw_temp);

    // Compensate pressure in Pa per datasheet algorithm.
    const float pressure_pa = CompensatePressure(raw_press);
    // Convert Pa to hPa to match telemetry expectations and existing code.
    data.pressure = pressure_pa * 0.01f;  // Pa -> hPa
    // Estimate altitude from pressure ratio for approximate height.
    data.altitude = PressureToAltitude(data.pressure, referencePressureHpa_);

    // Cache and return the latest sample so failures keep the last good data.
    data_ = data;
    lastReadSucceeded_ = true;
    return data_;
}

bool Barometer::RezeroReferencePressure() {
    if (!initialized_) {
        return false;
    }

    StartReferencePressureCapture(millis());
    return true;
}

void Barometer::StartReferencePressureCapture(uint32_t startMs) {
    referenceCaptureInProgress_ = true;
    referenceCaptureHasSample_ = false;
    referenceCaptureStartMs_ = startMs;
    lastReferenceSampleMs_ = startMs;
    referenceSampleCount_ = 0;
    nextReferenceSampleIndex_ = 0;
    referenceWindowPressures_.fill(0.0f);
}

Barometer::ReferenceCaptureStatus Barometer::ProcessReferencePressureSample(const Common::BarometerData& sample,
                                                                            uint32_t nowMs) {
    if (!referenceCaptureInProgress_) {
        return ReferenceCaptureStatus::kIdle;
    }

    if ((nowMs - referenceCaptureStartMs_) >= kReferenceCaptureTimeoutMs) {
        referenceCaptureInProgress_ = false;
        return ReferenceCaptureStatus::kTimedOut;
    }

    const bool sampleDue = !referenceCaptureHasSample_
        || ((nowMs - lastReferenceSampleMs_) >= kReferenceSamplePeriodMs);
    if (!sampleDue || sample.pressure <= 0.0f) {
        return ReferenceCaptureStatus::kPending;
    }

    referenceCaptureHasSample_ = true;
    lastReferenceSampleMs_ = nowMs;
    referenceWindowPressures_[nextReferenceSampleIndex_] = sample.pressure;
    nextReferenceSampleIndex_ = (nextReferenceSampleIndex_ + 1) % kReferenceWindowSampleCount;
    if (referenceSampleCount_ < kReferenceWindowSampleCount) {
        ++referenceSampleCount_;
    }

    if (referenceSampleCount_ < kReferenceWindowSampleCount) {
        return ReferenceCaptureStatus::kPending;
    }

    float pressureSumHpa = 0.0f;
    float minPressureHpa = referenceWindowPressures_[0];
    float maxPressureHpa = referenceWindowPressures_[0];
    for (size_t i = 0; i < kReferenceWindowSampleCount; ++i) {
        const float sampleHpa = referenceWindowPressures_[i];
        pressureSumHpa += sampleHpa;
        if (sampleHpa < minPressureHpa) {
            minPressureHpa = sampleHpa;
        }
        if (sampleHpa > maxPressureHpa) {
            maxPressureHpa = sampleHpa;
        }
    }

    if ((maxPressureHpa - minPressureHpa) > kReferenceStableSpanHpa) {
        return ReferenceCaptureStatus::kPending;
    }

    referencePressureHpa_ = pressureSumHpa / static_cast<float>(kReferenceWindowSampleCount);
    referenceCaptureInProgress_ = false;
    data_.altitude = 0.0f;
    return ReferenceCaptureStatus::kReady;
}

// Send a soft reset command to the sensor to clear prior configuration.
bool Barometer::SoftReset() {
    // Datasheet p39: "Triggers a reset, all user configuration settings are overwritten with their default state".
    // Write soft reset command to CMD register to trigger device reset.
    if (!device_.WriteRegister(kRegCmd, kCmdSoftReset)) {
        return false;
    }

    // Give the device time to reset before further access.
    delay(10);

    // Check command-ready status if available to avoid racing configuration.
    uint8_t status = 0;
    if (device_.ReadRegister(kRegStatus, &status) && (status & kStatusCmdReady)) {
        return true;
    }

    // Fallback delay; device should be ready after reset even if STATUS isn't updated.
    delay(10);
    return true;
}

// Read and convert calibration coefficients from NVM so compensation math is correct.
bool Barometer::ReadCalibration() {
    // Datasheet p28: "Trimming Coefficient listing in register map with size and sign attributes".
    // Read full calibration block in one burst to keep coefficient set consistent.
    uint8_t calib[21] = {};
    if (!device_.ReadRegisters(kRegCalibStart, calib, sizeof(calib), false)) {
        return false;
    }

    // Unpack raw coefficients by datasheet map so each term is in the right place.
    const uint16_t nvm_t1 = ReadU16(calib, 0);
    const uint16_t nvm_t2 = ReadU16(calib, 2);
    const int8_t nvm_t3 = static_cast<int8_t>(calib[4]);
    const int16_t nvm_p1 = ReadS16(calib, 5);
    const int16_t nvm_p2 = ReadS16(calib, 7);
    const int8_t nvm_p3 = static_cast<int8_t>(calib[9]);
    const int8_t nvm_p4 = static_cast<int8_t>(calib[10]);
    const uint16_t nvm_p5 = ReadU16(calib, 11);
    const uint16_t nvm_p6 = ReadU16(calib, 13);
    const int8_t nvm_p7 = static_cast<int8_t>(calib[15]);
    const int8_t nvm_p8 = static_cast<int8_t>(calib[16]);
    const int16_t nvm_p9 = ReadS16(calib, 17);
    const int8_t nvm_p10 = static_cast<int8_t>(calib[19]);
    const int8_t nvm_p11 = static_cast<int8_t>(calib[20]);

    // Datasheet p55: "Therefore the coefficient have to be converted into floating point numbers by the following formulas."
    // Convert raw coefficients into floating-point calibration terms required by formulas.
    calib_.par_t1 = static_cast<float>(nvm_t1) * std::ldexp(1.0f, 8);
    calib_.par_t2 = static_cast<float>(nvm_t2) * std::ldexp(1.0f, -30);
    calib_.par_t3 = static_cast<float>(nvm_t3) * std::ldexp(1.0f, -48);

    // Pressure coefficients have offsets and scales per datasheet to linearize output.
    calib_.par_p1 = (static_cast<float>(nvm_p1) - 16384.0f) * std::ldexp(1.0f, -20);
    calib_.par_p2 = (static_cast<float>(nvm_p2) - 16384.0f) * std::ldexp(1.0f, -29);
    calib_.par_p3 = static_cast<float>(nvm_p3) * std::ldexp(1.0f, -32);
    calib_.par_p4 = static_cast<float>(nvm_p4) * std::ldexp(1.0f, -37);
    calib_.par_p5 = static_cast<float>(nvm_p5) * 8.0f;
    calib_.par_p6 = static_cast<float>(nvm_p6) * std::ldexp(1.0f, -6);
    calib_.par_p7 = static_cast<float>(nvm_p7) * std::ldexp(1.0f, -8);
    calib_.par_p8 = static_cast<float>(nvm_p8) * std::ldexp(1.0f, -15);
    calib_.par_p9 = static_cast<float>(nvm_p9) * std::ldexp(1.0f, -48);
    calib_.par_p10 = static_cast<float>(nvm_p10) * std::ldexp(1.0f, -48);
    calib_.par_p11 = static_cast<float>(nvm_p11) * std::ldexp(1.0f, -65);

    // Calibration is ready for compensation routines.
    return true;
}

// Program sensor operating parameters so sampling matches mission needs.
bool Barometer::Configure() {
    // Datasheet p36: "The \"IF_CONF\" register controls the serial interface settings."
    // Leave interface settings at default (I2C, no watchdog) to avoid bus timeouts.
    if (!device_.WriteRegister(kRegIfConf, 0x00)) {
        return false;
    }
    // Datasheet p37: "The \"OSR\" register controls the oversampling settings for pressure and temperature measurements."
    // Pack temperature and pressure oversampling into OSR to set resolution.
    const uint8_t osr = static_cast<uint8_t>((kOsrTemp << 3) | kOsrPress);
    if (!device_.WriteRegister(kRegOsr, osr)) {
        return false;
    }
    // Datasheet p37: "The \"ODR\" register set the configuration of the output data rates by means of setting the subdivision/subsampling."
    // Set output data rate divider to control update frequency.
    if (!device_.WriteRegister(kRegOdr, kOdr)) {
        return false;
    }
    // Datasheet p39: "The \"CONFIG\" register controls the IIR filter coefficients."
    // Configure IIR filter strength to reduce noise without major lag.
    const uint8_t config = static_cast<uint8_t>(kIirFilter << 1);
    if (!device_.WriteRegister(kRegConfig, config)) {
        return false;
    }
    // Datasheet p36: "Enables the pressure sensor." / "Enables the temperature sensor." / "11 normal mode".
    // Enable sensors and enter normal mode for continuous measurements.
    if (!device_.WriteRegister(kRegPwrCtrl, kPwrCtrl)) {
        return false;
    }
    // Configuration complete.
    return true;
}

// Read raw 24-bit pressure and temperature from data registers for compensation.
bool Barometer::ReadRaw(uint32_t* raw_press, uint32_t* raw_temp) {
    // Validate output pointers to avoid null writes.
    if (raw_press == nullptr || raw_temp == nullptr) {
        return false;
    }

    // Datasheet p32: "The 24Bit pressure data is split and stored in three consecutive registers".
    // Datasheet p32: "The 24Bit temperature data is split and stored in three consecutive registers".
    // Burst read pressure + temperature to keep data coherent across registers.
    uint8_t data[6] = {};
    if (!device_.ReadRegisters(kRegDataStart, data, sizeof(data), false)) {
        return false;
    }

    // Assemble little-endian 24-bit pressure from three bytes.
    *raw_press = (static_cast<uint32_t>(data[2]) << 16)
        | (static_cast<uint32_t>(data[1]) << 8)
        | static_cast<uint32_t>(data[0]);
    // Assemble little-endian 24-bit temperature from three bytes.
    *raw_temp = (static_cast<uint32_t>(data[5]) << 16)
        | (static_cast<uint32_t>(data[4]) << 8)
        | static_cast<uint32_t>(data[3]);
    return true;
}

// Apply temperature compensation formula and update t_lin for pressure calc.
float Barometer::CompensateTemperature(uint32_t raw_temp) {
    // Datasheet p55: "Temperature compensation".
    // Compute delta from calibration reference to normalize raw ADC.
    const float partial_data1 = static_cast<float>(raw_temp) - calib_.par_t1;
    // Linear term of temperature compensation.
    const float partial_data2 = partial_data1 * calib_.par_t2;
    // Store t_lin for pressure compensation, as required by datasheet.
    calib_.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_.par_t3;
    // Return compensated temperature to caller.
    return calib_.t_lin;
}

// Apply pressure compensation formula using t_lin for final pressure.
float Barometer::CompensatePressure(uint32_t raw_press) {
    // Datasheet p56: "Pressure compensation".
    // First polynomial branch for temperature-dependent offset.
    float partial_data1 = calib_.par_p6 * calib_.t_lin;
    float partial_data2 = calib_.par_p7 * (calib_.t_lin * calib_.t_lin);
    float partial_data3 = calib_.par_p8 * (calib_.t_lin * calib_.t_lin * calib_.t_lin);
    float partial_out1 = calib_.par_p5 + partial_data1 + partial_data2 + partial_data3;

    // Second polynomial branch for temperature-scaled pressure.
    partial_data1 = calib_.par_p2 * calib_.t_lin;
    partial_data2 = calib_.par_p3 * (calib_.t_lin * calib_.t_lin);
    partial_data3 = calib_.par_p4 * (calib_.t_lin * calib_.t_lin * calib_.t_lin);
    float partial_out2 = static_cast<float>(raw_press)
        * (calib_.par_p1 + partial_data1 + partial_data2 + partial_data3);

    // Third polynomial branch for higher-order pressure corrections.
    partial_data1 = static_cast<float>(raw_press) * static_cast<float>(raw_press);
    partial_data2 = calib_.par_p9 + calib_.par_p10 * calib_.t_lin;
    partial_data3 = partial_data1 * partial_data2;
    const float partial_data4 = partial_data3
        + (static_cast<float>(raw_press) * static_cast<float>(raw_press) * static_cast<float>(raw_press))
        * calib_.par_p11;

    // Sum all branches to get compensated pressure in Pa.
    return partial_out1 + partial_out2 + partial_data4;
}

// Convert pressure in hPa to altitude using barometric formula for approximate height.
float Barometer::PressureToAltitude(float pressure_hpa, float reference_pressure_hpa) {
    // Guard against invalid pressures to avoid NaNs.
    if (pressure_hpa <= 0.0f || reference_pressure_hpa <= 0.0f) {
        return 0.0f;
    }

    // Compute ratio to the startup pressure so altitude is relative to the launch point.
    const float ratio = pressure_hpa / reference_pressure_hpa;
    // Compute relative altitude in meters using the standard atmosphere approximation.
    return 44330.0f * (1.0f - std::pow(ratio, 0.190294957f));
}
