// #include <cmath>
// #include <cstdlib>
// #include <iostream>

// #include "Wire.h"
// #include "Hardware/drivers/Magnetometer.h"

// // Pull the production implementation directly into the native test harness so
// // the register sequencing is tested without refactoring the driver.
// #include "../../src/Hardware/drivers/Magnetometer.cpp"

// namespace {
// constexpr uint8_t kTestMagAddress = 0x1E;
// constexpr uint8_t kTestMagWhoAmIReg = 0x4F;
// constexpr uint8_t kTestMagWhoAmIValue = 0x40;
// constexpr uint8_t kTestMagOutXLReg = 0x68;
// constexpr float kTestMagScale = 0.15f;

// bool SawWrite(uint8_t address, uint8_t reg, uint8_t value) {
//     for (const auto& entry : Wire.writeLog()) {
//         if (entry.address == address && entry.bytes.size() == 2
//             && entry.bytes[0] == reg && entry.bytes[1] == value) {
//             return true;
//         }
//     }
//     return false;
// }

// void SeedDeviceIds() {
//     Wire.setRegister(kTestMagAddress, kTestMagWhoAmIReg, kTestMagWhoAmIValue);
// }

// void SetMagRaw(int16_t x, int16_t y, int16_t z) {
//     Wire.setRegister(kTestMagAddress, kTestMagOutXLReg + 0, static_cast<uint8_t>(x & 0xFF));
//     Wire.setRegister(kTestMagAddress, kTestMagOutXLReg + 1, static_cast<uint8_t>((x >> 8) & 0xFF));
//     Wire.setRegister(kTestMagAddress, kTestMagOutXLReg + 2, static_cast<uint8_t>(y & 0xFF));
//     Wire.setRegister(kTestMagAddress, kTestMagOutXLReg + 3, static_cast<uint8_t>((y >> 8) & 0xFF));
//     Wire.setRegister(kTestMagAddress, kTestMagOutXLReg + 4, static_cast<uint8_t>(z & 0xFF));
//     Wire.setRegister(kTestMagAddress, kTestMagOutXLReg + 5, static_cast<uint8_t>((z >> 8) & 0xFF));
// }
// }  // namespace

// void setUp() {
//     Wire.clear();
//     SeedDeviceIds();
// }

// [[noreturn]] void Fail(const char* message) {
//     std::cerr << "FAIL: " << message << '\n';
//     std::exit(1);
// }

// void AssertTrue(bool condition, const char* message) {
//     if (!condition) {
//         Fail(message);
//     }
// }

// void AssertFloatNear(float expected, float actual, float tol, const char* message) {
//     if (std::fabs(expected - actual) > tol) {
//         std::cerr << "FAIL: " << message << " expected=" << expected << " actual=" << actual << '\n';
//         std::exit(1);
//     }
// }

// void test_magnetometer_initialize_programs_expected_registers() {
//     setUp();
//     Magnetometer magnetometer;
//     AssertTrue(magnetometer.Initialize(), "Magnetometer initialize failed");

//     AssertTrue(SawWrite(kTestMagAddress, 0x60, 0x20), "CFG_REG_A reset not issued");
//     AssertTrue(SawWrite(kTestMagAddress, 0x60, 0x88), "CFG_REG_A not set to 50 Hz continuous mode");
//     AssertTrue(SawWrite(kTestMagAddress, 0x61, 0x00), "CFG_REG_B not programmed");
//     AssertTrue(SawWrite(kTestMagAddress, 0x62, 0x20), "CFG_REG_C BDU not enabled");
// }

// void test_magnetometer_read_converts_raw_xyz() {
//     setUp();
//     Magnetometer magnetometer;
//     AssertTrue(magnetometer.Initialize(), "Magnetometer initialize failed");

//     SetMagRaw(10, 20, 30);
//     const Common::MagData mag = magnetometer.ReadData();
//     AssertFloatNear(10.0f * kTestMagScale, mag.magX, 1e-5f, "Mag X conversion mismatch");
//     AssertFloatNear(20.0f * kTestMagScale, mag.magY, 1e-5f, "Mag Y conversion mismatch");
//     AssertFloatNear(30.0f * kTestMagScale, mag.magZ, 1e-5f, "Mag Z conversion mismatch");
// }

// void test_initialize_fails_when_probe_fails() {
//     setUp();
//     Wire.setRegister(kTestMagAddress, kTestMagWhoAmIReg, 0x00);

//     Magnetometer magnetometer;
//     AssertTrue(!magnetometer.Initialize(), "Initialize should fail when magnetometer WHO_AM_I is wrong");
//     const Magnetometer::Diagnostics diagnostics = magnetometer.GetDiagnostics();
//     AssertTrue(diagnostics.initFailureCount == 1, "Init failure should increment diagnostics");
// }

// void test_read_failure_returns_last_good_data_and_tracks_diagnostics() {
//     setUp();
//     Magnetometer magnetometer;
//     AssertTrue(magnetometer.Initialize(), "Magnetometer initialize failed");

//     SetMagRaw(10, 20, 30);
//     const Common::MagData first = magnetometer.ReadData();
//     Wire.setReadFailure(kTestMagAddress, kTestMagOutXLReg, true);

//     const Common::MagData second = magnetometer.ReadData();
//     AssertFloatNear(first.magX, second.magX, 1e-5f, "Read failure should return last good X");
//     AssertFloatNear(first.magY, second.magY, 1e-5f, "Read failure should return last good Y");
//     AssertFloatNear(first.magZ, second.magZ, 1e-5f, "Read failure should return last good Z");

//     const Magnetometer::Diagnostics diagnostics = magnetometer.GetDiagnostics();
//     AssertTrue(diagnostics.readFailureCount == 1, "Read failure should increment diagnostics");
//     AssertTrue(diagnostics.staleReturnCount == 1, "Stale return count should increment on read failure");
// }

// int main(int, char**) {
//     test_magnetometer_initialize_programs_expected_registers();
//     test_magnetometer_read_converts_raw_xyz();
//     test_initialize_fails_when_probe_fails();
//     test_read_failure_returns_last_good_data_and_tracks_diagnostics();
//     std::cout << "PASS: Magnetometer service testbench\n";
//     return 0;
// }
