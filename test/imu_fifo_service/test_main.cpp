#include <array>
#include <cstdlib>
#include <cmath>
#include <iostream>

#include "Wire.h"
#include "Hardware/drivers/Imu.h"

// Pull the production implementation directly into the native test harness so
// the FIFO service logic is exercised without refactoring the driver.
#include "../../src/Hardware/drivers/Imu.cpp"

namespace {
constexpr uint8_t kTestImuAddress = 0x6A;
constexpr uint8_t kTestMagAddress = 0x1E;
constexpr uint8_t kTestWhoAmIReg = 0x0F;
constexpr uint8_t kTestWhoAmIValue = 0x6B;
constexpr uint8_t kTestMagWhoAmIReg = 0x4F;
constexpr uint8_t kTestMagWhoAmIValue = 0x40;
constexpr uint8_t kTestMagOutXLReg = 0x68;
constexpr float kTestAccelScale = 0.488f * 0.001f * 9.80665f;
constexpr float kTestGyroScale = 70.0f * 0.001f * 0.017453292519943295f;
constexpr float kTestMagScale = 0.15f;

std::array<uint8_t, 7> MakeFifoWord(uint8_t tag, int16_t x, int16_t y, int16_t z) {
    return {
        static_cast<uint8_t>(tag << 3),
        static_cast<uint8_t>(x & 0xFF),
        static_cast<uint8_t>((x >> 8) & 0xFF),
        static_cast<uint8_t>(y & 0xFF),
        static_cast<uint8_t>((y >> 8) & 0xFF),
        static_cast<uint8_t>(z & 0xFF),
        static_cast<uint8_t>((z >> 8) & 0xFF),
    };
}

bool SawWrite(uint8_t address, uint8_t reg, uint8_t value) {
    for (const auto& entry : Wire.writeLog()) {
        if (entry.address == address && entry.bytes.size() == 2
            && entry.bytes[0] == reg && entry.bytes[1] == value) {
            return true;
        }
    }
    return false;
}

void SeedDeviceIds() {
    Wire.setRegister(kTestImuAddress, kTestWhoAmIReg, kTestWhoAmIValue);
    Wire.setRegister(kTestMagAddress, kTestMagWhoAmIReg, kTestMagWhoAmIValue);
}
}  // namespace

void setUp() {
    Wire.clear();
    SeedDeviceIds();
}

[[noreturn]] void Fail(const char* message) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
}

void AssertTrue(bool condition, const char* message) {
    if (!condition) {
        Fail(message);
    }
}

void AssertEqual(size_t expected, size_t actual, const char* message) {
    if (expected != actual) {
        std::cerr << "FAIL: " << message << " expected=" << expected << " actual=" << actual << '\n';
        std::exit(1);
    }
}

void AssertFloatNear(float expected, float actual, float tol, const char* message) {
    if (std::fabs(expected - actual) > tol) {
        std::cerr << "FAIL: " << message << " expected=" << expected << " actual=" << actual << '\n';
        std::exit(1);
    }
}

void SetMagRaw(int16_t x, int16_t y, int16_t z) {
    Wire.setRegister(kTestMagAddress, kTestMagOutXLReg + 0, static_cast<uint8_t>(x & 0xFF));
    Wire.setRegister(kTestMagAddress, kTestMagOutXLReg + 1, static_cast<uint8_t>((x >> 8) & 0xFF));
    Wire.setRegister(kTestMagAddress, kTestMagOutXLReg + 2, static_cast<uint8_t>(y & 0xFF));
    Wire.setRegister(kTestMagAddress, kTestMagOutXLReg + 3, static_cast<uint8_t>((y >> 8) & 0xFF));
    Wire.setRegister(kTestMagAddress, kTestMagOutXLReg + 4, static_cast<uint8_t>(z & 0xFF));
    Wire.setRegister(kTestMagAddress, kTestMagOutXLReg + 5, static_cast<uint8_t>((z >> 8) & 0xFF));
}

void test_imu_initialize_programs_fifo_for_416hz_batches() {
    setUp();
    IMU imu;
    AssertTrue(imu.Initialize(), "IMU initialize failed");

    AssertTrue(SawWrite(kTestImuAddress, 0x10, 0x64), "CTRL1_XL not set to 416 Hz / 16 g");
    AssertTrue(SawWrite(kTestImuAddress, 0x11, 0x6C), "CTRL2_G not set to 416 Hz / 2000 dps");
    AssertTrue(SawWrite(kTestImuAddress, 0x07, 0x08), "FIFO watermark LSB not configured");
    AssertTrue(SawWrite(kTestImuAddress, 0x08, 0x00), "FIFO watermark MSB not configured");
    AssertTrue(SawWrite(kTestImuAddress, 0x09, 0x66), "FIFO batch rates not set to 416 Hz");
    AssertTrue(SawWrite(kTestImuAddress, 0x0A, 0x00), "FIFO bypass reset not issued");
    AssertTrue(SawWrite(kTestImuAddress, 0x0A, 0x06), "FIFO continuous mode not enabled");
    AssertTrue(!SawWrite(kTestImuAddress, 0x0D, 0x08), "INT1 FIFO threshold should not be enabled on this PCB");
}

void test_fifo_service_reconstructs_four_ordered_imu_samples() {
    setUp();
    IMU imu;
    AssertTrue(imu.Initialize(), "IMU initialize failed");

    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x01, 100, 200, 300));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x02, 400, 500, 600));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x01, 110, 210, 310));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x02, 410, 510, 610));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x01, 120, 220, 320));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x02, 420, 520, 620));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x01, 130, 230, 330));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x02, 430, 530, 630));

    std::array<Common::IMUData, IMU::kFifoWatermarkPairs> samples{};
    const size_t count = imu.ReadFifoSamples(samples.data(), samples.size());

    AssertEqual(IMU::kFifoWatermarkPairs, count, "FIFO sample count mismatch");
    AssertFloatNear(100.0f * kTestGyroScale, samples[0].gyroVel.x, 1e-5f, "First gyro sample mismatch");
    AssertFloatNear(400.0f * kTestAccelScale, samples[0].acceleration.x, 1e-5f, "First accel sample mismatch");
    AssertFloatNear(130.0f * kTestGyroScale, samples[3].gyroVel.x, 1e-5f, "Last gyro sample mismatch");
    AssertFloatNear(430.0f * kTestAccelScale, samples[3].acceleration.x, 1e-5f, "Last accel sample mismatch");
}

void test_fifo_service_skips_non_imu_tags_without_emitting_partial_samples() {
    setUp();
    IMU imu;
    AssertTrue(imu.Initialize(), "IMU initialize failed");

    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x03, 1, 2, 3));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x01, 100, 200, 300));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x05, 4, 5, 6));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x02, 400, 500, 600));

    std::array<Common::IMUData, IMU::kFifoWatermarkPairs> samples{};
    const size_t count = imu.ReadFifoSamples(samples.data(), samples.size());

    AssertEqual(1, count, "Non-IMU tags should not emit partial samples");
    AssertFloatNear(100.0f * kTestGyroScale, samples[0].gyroVel.x, 1e-5f, "Filtered gyro sample mismatch");
    AssertFloatNear(400.0f * kTestAccelScale, samples[0].acceleration.x, 1e-5f, "Filtered accel sample mismatch");
}

void test_fifo_service_returns_zero_for_uninitialized_or_invalid_requests() {
    setUp();
    IMU imu;
    std::array<Common::IMUData, IMU::kFifoWatermarkPairs> samples{};

    AssertEqual(0, imu.ReadFifoSamples(samples.data(), samples.size()), "Uninitialized driver should not read FIFO");
    AssertTrue(imu.Initialize(), "IMU initialize failed");
    AssertEqual(0, imu.ReadFifoSamples(nullptr, samples.size()), "Null output buffer should return zero");
    AssertEqual(0, imu.ReadFifoSamples(samples.data(), 0), "Zero maxSamples should return zero");
    AssertEqual(0, imu.ReadFifoSamples(samples.data(), samples.size()), "Empty FIFO should return zero");
}

void test_fifo_service_caps_burst_and_leaves_remaining_words_in_fifo() {
    setUp();
    IMU imu;
    AssertTrue(imu.Initialize(), "IMU initialize failed");

    for (int i = 0; i < 6; ++i) {
        Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x01, static_cast<int16_t>(100 + i), 0, 0));
        Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x02, static_cast<int16_t>(400 + i), 0, 0));
    }

    std::array<Common::IMUData, 2> samples{};
    const size_t count = imu.ReadFifoSamples(samples.data(), samples.size());

    AssertEqual(2, count, "FIFO burst should cap at caller capacity");
    AssertEqual(8, Wire.fifoWordCount(kTestImuAddress), "Remaining FIFO words should stay queued");
    AssertFloatNear(100.0f * kTestGyroScale, samples[0].gyroVel.x, 1e-5f, "Capped burst first sample mismatch");
    AssertFloatNear(401.0f * kTestAccelScale, samples[1].acceleration.x, 1e-5f, "Capped burst second sample mismatch");
}

void test_fifo_service_drops_incomplete_pair_and_consumes_words() {
    setUp();
    IMU imu;
    AssertTrue(imu.Initialize(), "IMU initialize failed");

    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x01, 100, 200, 300));

    std::array<Common::IMUData, IMU::kFifoWatermarkPairs> samples{};
    const size_t count = imu.ReadFifoSamples(samples.data(), samples.size());

    AssertEqual(0, count, "Incomplete FIFO pair should not emit a sample");
    AssertEqual(0, Wire.fifoWordCount(kTestImuAddress), "Consumed incomplete word should not remain in FIFO");
}

void test_fifo_diagnostics_track_status_bits_and_decode_anomalies() {
    setUp();
    IMU imu;
    AssertTrue(imu.Initialize(), "IMU initialize failed");

    Wire.setFifoStatus2Bits(kTestImuAddress, static_cast<uint8_t>((1u << 6) | (1u << 5)));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x03, 1, 2, 3));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x01, 100, 200, 300));

    std::array<Common::IMUData, IMU::kFifoWatermarkPairs> samples{};
    const size_t count = imu.ReadFifoSamples(samples.data(), samples.size());
    const IMU::FifoDiagnostics diagnostics = imu.GetFifoDiagnostics();

    AssertEqual(0, count, "Unexpected tags and dangling samples should not emit IMU data");
    AssertEqual(1, diagnostics.overrunCount, "FIFO overrun count mismatch");
    AssertEqual(1, diagnostics.nearFullCount, "FIFO full count mismatch");
    AssertEqual(1, diagnostics.unexpectedTagCount, "Unexpected FIFO tag count mismatch");
    AssertEqual(1, diagnostics.truncatedPairCount, "Truncated FIFO pair count mismatch");
}

void test_fifo_diagnostics_can_be_cleared() {
    setUp();
    IMU imu;
    AssertTrue(imu.Initialize(), "IMU initialize failed");

    Wire.setFifoStatus2Bits(kTestImuAddress, static_cast<uint8_t>(1u << 6));
    Wire.pushFifoWord(kTestImuAddress, MakeFifoWord(0x01, 100, 200, 300));

    std::array<Common::IMUData, IMU::kFifoWatermarkPairs> samples{};
    (void)imu.ReadFifoSamples(samples.data(), samples.size());
    imu.ClearFifoDiagnostics();
    const IMU::FifoDiagnostics diagnostics = imu.GetFifoDiagnostics();

    AssertEqual(0, diagnostics.overrunCount, "FIFO overrun count should clear");
    AssertEqual(0, diagnostics.nearFullCount, "FIFO full count should clear");
    AssertEqual(0, diagnostics.unexpectedTagCount, "Unexpected tag count should clear");
    AssertEqual(0, diagnostics.truncatedPairCount, "Truncated pair count should clear");
}

void test_direct_read_uses_live_registers_and_last_mag_data() {
    setUp();
    IMU imu;
    AssertTrue(imu.Initialize(), "IMU initialize failed");

    SetMagRaw(10, 20, 30);
    const Common::MagData magData = imu.ReadMagData();
    AssertFloatNear(10.0f * kTestMagScale, magData.magX, 1e-5f, "Mag X conversion mismatch");
    AssertFloatNear(20.0f * kTestMagScale, magData.magY, 1e-5f, "Mag Y conversion mismatch");
    AssertFloatNear(30.0f * kTestMagScale, magData.magZ, 1e-5f, "Mag Z conversion mismatch");

    Wire.setRegister(kTestImuAddress, 0x22, 100 & 0xFF);
    Wire.setRegister(kTestImuAddress, 0x23, (100 >> 8) & 0xFF);
    Wire.setRegister(kTestImuAddress, 0x24, 0);
    Wire.setRegister(kTestImuAddress, 0x25, 0);
    Wire.setRegister(kTestImuAddress, 0x26, 0);
    Wire.setRegister(kTestImuAddress, 0x27, 0);
    Wire.setRegister(kTestImuAddress, 0x28, 50 & 0xFF);
    Wire.setRegister(kTestImuAddress, 0x29, (50 >> 8) & 0xFF);
    Wire.setRegister(kTestImuAddress, 0x2A, 0);
    Wire.setRegister(kTestImuAddress, 0x2B, 0);
    Wire.setRegister(kTestImuAddress, 0x2C, 0);
    Wire.setRegister(kTestImuAddress, 0x2D, 0);

    const Common::IMUData direct = imu.ReadData();
    AssertFloatNear(100.0f * kTestGyroScale, direct.gyroVel.x, 1e-5f, "Direct gyro conversion mismatch");
    AssertFloatNear(50.0f * kTestAccelScale, direct.acceleration.x, 1e-5f, "Direct accel conversion mismatch");
}

void test_initialize_fails_when_magnetometer_probe_fails() {
    setUp();
    Wire.setRegister(kTestMagAddress, kTestMagWhoAmIReg, 0x00);

    IMU imu;
    AssertTrue(!imu.Initialize(), "Initialize should fail when magnetometer WHO_AM_I is wrong");
}

int main(int, char**) {
    test_imu_initialize_programs_fifo_for_416hz_batches();
    test_fifo_service_reconstructs_four_ordered_imu_samples();
    test_fifo_service_skips_non_imu_tags_without_emitting_partial_samples();
    test_fifo_service_returns_zero_for_uninitialized_or_invalid_requests();
    test_fifo_service_caps_burst_and_leaves_remaining_words_in_fifo();
    test_fifo_service_drops_incomplete_pair_and_consumes_words();
    test_fifo_diagnostics_track_status_bits_and_decode_anomalies();
    test_fifo_diagnostics_can_be_cleared();
    test_direct_read_uses_live_registers_and_last_mag_data();
    test_initialize_fails_when_magnetometer_probe_fails();
    std::cout << "PASS: IMU FIFO service testbench\n";
    return 0;
}
