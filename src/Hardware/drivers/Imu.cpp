#include "Hardware/drivers/Imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <algorithm>
#include <cmath>

namespace {
// Local register map + scaling constants live here to keep the driver readable.
constexpr uint8_t kImuAddressLow = 0x6A;
constexpr uint8_t kImuAddressHigh = 0x6B;
constexpr uint8_t kMagAddress = 0x1E;

constexpr uint8_t kImuWhoAmIReg = 0x0F;
constexpr uint8_t kImuWhoAmIValue = 0x6B;
constexpr uint8_t kImuCtrl1XlReg = 0x10;
constexpr uint8_t kImuCtrl2GReg = 0x11;
constexpr uint8_t kImuCtrl3CReg = 0x12;
constexpr uint8_t kFifoCtrl1Reg = 0x07;
constexpr uint8_t kFifoCtrl2Reg = 0x08;
constexpr uint8_t kFifoCtrl3Reg = 0x09;
constexpr uint8_t kFifoCtrl4Reg = 0x0A;
constexpr uint8_t kFifoStatus1Reg = 0x3A;
constexpr uint8_t kFifoStatus2Reg = 0x3B;
constexpr uint8_t kFifoDataOutTagReg = 0x78;
constexpr uint8_t kImuOutXLGReg = 0x22;

constexpr uint8_t kMagWhoAmIReg = 0x4F;
constexpr uint8_t kMagWhoAmIValue = 0x40;
constexpr uint8_t kMagCfgAReg = 0x60;
constexpr uint8_t kMagCfgBReg = 0x61;
constexpr uint8_t kMagCfgCReg = 0x62;
constexpr uint8_t kMagOutXLReg = 0x68;

constexpr uint8_t kImuCtrl3CSwReset = 0x01;
constexpr uint8_t kImuCtrl3CIfInc = 0x04;
constexpr uint8_t kImuCtrl3CBdu = 0x40;

constexpr uint8_t kAccelOdrBits = 0x06;  // 416 Hz.
constexpr uint8_t kAccelFsBits = 0x01;   // +/- 16 g.
constexpr float kAccelSensitivityMg = 0.488f;

constexpr uint8_t kGyroOdrBits = 0x06;   // 416 Hz.
constexpr uint8_t kGyroFsBits = 0x03;    // +/- 2000 dps.
constexpr float kGyroSensitivityMdps = 70.0f;

constexpr uint8_t kFifoBdr416Hz = 0x06;
constexpr uint8_t kFifoModeBypass = 0x00;
constexpr uint8_t kFifoModeContinuous = 0x06;
constexpr uint16_t kFifoWatermarkWords = static_cast<uint16_t>(IMU::kFifoWatermarkPairs * 2);
constexpr uint8_t kFifoTagGyro = 0x01;
constexpr uint8_t kFifoTagAccel = 0x02;
constexpr uint8_t kFifoOverrunStatusBit = 1u << 6;
constexpr uint8_t kFifoFullStatusBit = 1u << 5;

constexpr uint8_t kMagCfgAReset = 0x20;
constexpr uint8_t kMagCfgA = 0x88;  // Temp comp + 50 Hz + continuous mode.
constexpr uint8_t kMagCfgB = 0x00;
constexpr uint8_t kMagCfgC = 0x20;  // BDU enabled.

constexpr float kGravityMps2 = 9.80665f;
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kMagMicroTeslaPerLsb = 0.15f;  // 1.5 mG/LSB.

struct RawImuSample {
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t ax;
    int16_t ay;
    int16_t az;
};

struct RawMagSample {
    int16_t mx;
    int16_t my;
    int16_t mz;
};

// Separate raw read helpers keep register layout isolated from math conversions.
bool ReadImuRaw(const I2CDevice& device, RawImuSample* sample) {
    // Read gyro + accel in a single burst for coherent data.
    uint8_t raw[12] = {};
    // TODO: Double check w/ datasheet.
    if (!device.ReadRegisters(kImuOutXLGReg, raw, sizeof(raw), false)) {
        // If the I2C read fails, keep previous values upstream.
        return false;
    }

    // Combine low/high bytes into signed 16-bit values (sensor is little-endian).
    sample->gx = static_cast<int16_t>((raw[1] << 8) | raw[0]);
    sample->gy = static_cast<int16_t>((raw[3] << 8) | raw[2]);
    sample->gz = static_cast<int16_t>((raw[5] << 8) | raw[4]);
    sample->ax = static_cast<int16_t>((raw[7] << 8) | raw[6]);
    sample->ay = static_cast<int16_t>((raw[9] << 8) | raw[8]);
    sample->az = static_cast<int16_t>((raw[11] << 8) | raw[10]);
    return true;
}

bool ReadMagRaw(const I2CDevice& device, RawMagSample* sample) {
    // Read magnetometer XYZ in one burst with auto-increment.
    uint8_t raw[6] = {};
    if (!device.ReadRegisters(kMagOutXLReg, raw, sizeof(raw), false)) {
        return false;
    }

    // Combine low/high bytes into signed 16-bit values (sensor is little-endian).
    sample->mx = static_cast<int16_t>((raw[1] << 8) | raw[0]);
    sample->my = static_cast<int16_t>((raw[3] << 8) | raw[2]);
    sample->mz = static_cast<int16_t>((raw[5] << 8) | raw[4]);
    return true;
}

bool ReadFifoWord(const I2CDevice& device, uint8_t* tag, RawImuSample* sample) {
    uint8_t raw[7] = {};
    if (!device.ReadRegisters(kFifoDataOutTagReg, raw, sizeof(raw), false)) {
        return false;
    }

    *tag = static_cast<uint8_t>((raw[0] >> 3) & 0x1F);
    sample->gx = sample->gy = sample->gz = 0;
    sample->ax = sample->ay = sample->az = 0;

    const int16_t x = static_cast<int16_t>((raw[2] << 8) | raw[1]);
    const int16_t y = static_cast<int16_t>((raw[4] << 8) | raw[3]);
    const int16_t z = static_cast<int16_t>((raw[6] << 8) | raw[5]);

    if (*tag == kFifoTagGyro) {
        sample->gx = x;
        sample->gy = y;
        sample->gz = z;
        return true;
    }

    if (*tag == kFifoTagAccel) {
        sample->ax = x;
        sample->ay = y;
        sample->az = z;
        return true;
    }

    return true;
}

bool ReadFifoStatus(const I2CDevice& device, uint16_t* unreadWords, uint8_t* status2) {
    if (unreadWords == nullptr || status2 == nullptr) {
        return false;
    }

    uint8_t status[2] = {};
    if (!device.ReadRegisters(kFifoStatus1Reg, status, sizeof(status), false)) {
        return false;
    }

    *unreadWords = static_cast<uint16_t>(status[0]) | (static_cast<uint16_t>(status[1] & 0x03) << 8);
    *status2 = status[1];
    return true;
}

// Convert raw counts to SI units so the rest of the stack is unit-consistent.
Common::Vector3 ConvertAccel(const RawImuSample& sample) {
    // Scale converts mg/LSB to m/s^2 using standard gravity.
    const float scale = (kAccelSensitivityMg * 0.001f) * kGravityMps2;
    return Common::Vector3{
        sample.ax * scale,
        sample.ay * scale,
        sample.az * scale,
    };
}

Common::Vector3 ConvertGyro(const RawImuSample& sample) {
    // Scale converts mdps/LSB to rad/s for downstream math.
    const float scale = (kGyroSensitivityMdps * 0.001f) * kDegToRad;
    return Common::Vector3{
        sample.gx * scale,
        sample.gy * scale,
        sample.gz * scale,
    };
}

Common::Vector3 ConvertMag(const RawMagSample& sample) {
    // Sensor is factory calibrated to 1.5 mG/LSB.
    return Common::Vector3{
        sample.mx * kMagMicroTeslaPerLsb,
        sample.my * kMagMicroTeslaPerLsb,
        sample.mz * kMagMicroTeslaPerLsb,
    };
}

Common::IMUData BuildImuData(const RawImuSample& imuRaw,
                             const Common::MagData& magData) {
    Common::Vector3 accel = ConvertAccel(imuRaw);
    Common::Vector3 gyro = ConvertGyro(imuRaw);

 
    Common::IMUData data{};
    data.acceleration = accel;
    data.gyroVel = gyro;

    return data;
}

}  // namespace

bool IMU::Initialize() {
    // Reset cached data and state before touching hardware.
    data_ = {};
    magData_ = {};
    fifoDiagnostics_ = {};
    accelBias_ = {};
    gyroBias_ = {};
    magBias_ = {};
    initialized_ = false;

    // IMU uses I2C for both chips; keep setup centralized here for now.
    Wire.begin();
    Wire.setClock(Common::I2C_BAUDRATE);

    // WHO_AM_I read validates we are talking to the expected device.
    uint8_t whoami = 0;
    // Probe both possible IMU addresses so boards can strap SA0 either way.
    imuDevice_.setAddress(kImuAddressLow);
    if (!imuDevice_.ReadRegister(kImuWhoAmIReg, &whoami) || whoami != kImuWhoAmIValue) {
        // Retry on the alternate address if the first probe fails.
        imuDevice_.setAddress(kImuAddressHigh);
        if (!imuDevice_.ReadRegister(kImuWhoAmIReg, &whoami) || whoami != kImuWhoAmIValue) {
            return false;
        }
    }

    // Reset first to guarantee known register state before config.
    // Soft reset clears any previous configuration.
    if (!imuDevice_.WriteRegister(kImuCtrl3CReg, kImuCtrl3CSwReset)) {
        return false;
    }
    // Datasheet requires a small delay for reset to complete.
    delay(10);
    // Enable block data update and address auto-increment.
    if (!imuDevice_.WriteRegister(kImuCtrl3CReg, kImuCtrl3CBdu | kImuCtrl3CIfInc)) {
        return false;
    }

    // ODR/FS choices are centralized constants for quick tuning.
    // Pack ODR and full-scale settings into control registers.
    // 01000100b
    const uint8_t accelConfig = static_cast<uint8_t>((kAccelOdrBits << 4) | (kAccelFsBits << 2));
    const uint8_t gyroConfig = static_cast<uint8_t>((kGyroOdrBits << 4) | (kGyroFsBits << 2));
    if (!imuDevice_.WriteRegister(kImuCtrl1XlReg, accelConfig)) {
        return false;
    }
    if (!imuDevice_.WriteRegister(kImuCtrl2GReg, gyroConfig)) {
        return false;
    }

    // Reset FIFO state before enabling continuous batching, then configure both
    // accel and gyro to batch into FIFO at the sensor ODR. The watermark counts
    // FIFO words, so 8 words corresponds to 4 accel+gyro pairs.
    if (!imuDevice_.WriteRegister(kFifoCtrl4Reg, kFifoModeBypass)) {
        return false;
    }
    if (!imuDevice_.WriteRegister(kFifoCtrl1Reg, static_cast<uint8_t>(kFifoWatermarkWords & 0xFF))) {
        return false;
    }
    if (!imuDevice_.WriteRegister(kFifoCtrl2Reg, static_cast<uint8_t>((kFifoWatermarkWords >> 8) & 0x01))) {
        return false;
    }
    if (!imuDevice_.WriteRegister(kFifoCtrl3Reg, static_cast<uint8_t>((kFifoBdr416Hz << 4) | kFifoBdr416Hz))) {
        return false;
    }
    if (!imuDevice_.WriteRegister(kFifoCtrl4Reg, kFifoModeContinuous)) {
        return false;
    }

    // Magnetometer is a separate die on the same bus.
    // Magnetometer has a fixed I2C address on this board.
    magDevice_.setAddress(kMagAddress);
    // Confirm the magnetometer is present before configuring it.
    if (!magDevice_.ReadRegister(kMagWhoAmIReg, &whoami) || whoami != kMagWhoAmIValue) {
        return false;
    }

    // Reset the magnetometer before applying settings.
    if (!magDevice_.WriteRegister(kMagCfgAReg, kMagCfgAReset)) {
        return false;
    }
    delay(10);
    // Configure ODR, continuous mode, and temperature compensation.
    if (!magDevice_.WriteRegister(kMagCfgAReg, kMagCfgA)) {
        return false;
    }
    // Optional filter/offset settings remain at defaults for now.
    if (!magDevice_.WriteRegister(kMagCfgBReg, kMagCfgB)) {
        return false;
    }
    // Enable block data update to avoid mixed high/low reads.
    if (!magDevice_.WriteRegister(kMagCfgCReg, kMagCfgC)) {
        return false;
    }

    // Mark initialization complete so reads can proceed.
    initialized_ = true;
    return true;
}

Common::IMUData IMU::ReadData() {
    if (!initialized_) {
        return data_;
    }

    // Keep last good data if a read fails, avoiding sudden zero spikes.
    RawImuSample imuRaw{};
    if (!ReadImuRaw(imuDevice_, &imuRaw)) {
        return data_;
    }

    data_ = BuildImuData(imuRaw, magData_);
    return data_;
}

size_t IMU::ReadFifoSamples(Common::IMUData* outSamples, size_t maxSamples) {
    if (!initialized_ || outSamples == nullptr || maxSamples == 0) {
        return 0;
    }

    uint16_t unreadWords = 0;
    uint8_t fifoStatus2 = 0;
    if (!ReadFifoStatus(imuDevice_, &unreadWords, &fifoStatus2)) {
        return 0;
    }

    if ((fifoStatus2 & kFifoOverrunStatusBit) != 0) {
        ++fifoDiagnostics_.overrunCount;
    }
    if ((fifoStatus2 & kFifoFullStatusBit) != 0) {
        ++fifoDiagnostics_.nearFullCount;
    }

    if (unreadWords == 0) {
        return 0;
    }

    RawImuSample pending{};
    bool haveGyro = false;
    bool haveAccel = false;
    size_t samplesRead = 0;

    const uint16_t wordsToRead = std::min<uint16_t>(unreadWords, static_cast<uint16_t>(maxSamples * 2));
    for (uint16_t i = 0; i < wordsToRead; ++i) {
        uint8_t tag = 0;
        RawImuSample fifoWord{};
        if (!ReadFifoWord(imuDevice_, &tag, &fifoWord)) {
            break;
        }

        if (tag == kFifoTagGyro) {
            pending.gx = fifoWord.gx;
            pending.gy = fifoWord.gy;
            pending.gz = fifoWord.gz;
            haveGyro = true;
        } else if (tag == kFifoTagAccel) {
            pending.ax = fifoWord.ax;
            pending.ay = fifoWord.ay;
            pending.az = fifoWord.az;
            haveAccel = true;
        } else {
            ++fifoDiagnostics_.unexpectedTagCount;
            continue;
        }

        if (haveGyro && haveAccel) {
            outSamples[samplesRead] = BuildImuData(pending, magData_);
            data_ = outSamples[samplesRead];
            ++samplesRead;
            haveGyro = false;
            haveAccel = false;
            if (samplesRead >= maxSamples) {
                break;
            }
        }
    }

    if (haveGyro || haveAccel) {
        ++fifoDiagnostics_.truncatedPairCount;
    }

    return samplesRead;
}

Common::MagData IMU::ReadMagData() {
    if (!initialized_) {
        return magData_;
    }

    RawMagSample magRaw{};
    if (!ReadMagRaw(magDevice_, &magRaw)) {
        return magData_;
    }

    Common::Vector3 mag = ConvertMag(magRaw);
    magData_ = Common::MagData{mag.x, mag.y, mag.z};
    return magData_;
}

IMU::FifoDiagnostics IMU::GetFifoDiagnostics() const {
    return fifoDiagnostics_;
}

void IMU::ClearFifoDiagnostics() {
    fifoDiagnostics_ = {};
}

void IMU::Calibrate() {
    if (!initialized_) {
        return;
    }

    // Bias calibration assumes the device is stationary.
    constexpr int kSamples = 200;
    Common::Vector3 accelSum{};
    Common::Vector3 gyroSum{};
    int sampleCount = 0;

    for (int i = 0; i < kSamples; ++i) {
        // Read raw data each iteration; skip samples that fail.
        RawImuSample imuRaw{};
        if (!ReadImuRaw(imuDevice_, &imuRaw)) {
            continue;
        }

        Common::Vector3 accel = ConvertAccel(imuRaw);
        Common::Vector3 gyro = ConvertGyro(imuRaw);

        accelSum.x += accel.x;
        accelSum.y += accel.y;
        accelSum.z += accel.z;

        gyroSum.x += gyro.x;
        gyroSum.y += gyro.y;
        gyroSum.z += gyro.z;

        // Track the number of valid samples used in the average.
        ++sampleCount;
        delay(5);
    }

    // Guard against divide-by-zero if all reads failed.
    if (sampleCount == 0) {
        return;
    }

    // Bias is the mean of the stationary readings.
    accelBias_ = Common::Vector3{
        accelSum.x / sampleCount,
        accelSum.y / sampleCount,
        (accelSum.z / sampleCount) - kGravityMps2,
    };

    // Gyro bias should converge near zero for a stationary device.
    gyroBias_ = Common::Vector3{
        gyroSum.x / sampleCount,
        gyroSum.y / sampleCount,
        gyroSum.z / sampleCount,
    };
}
