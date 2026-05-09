#include <cmath>
#include <cstdlib>
#include <iostream>

#include "../support/Wire.h"
#include "Hardware/drivers/TeseoGPS.h"

// Pull the production implementation directly into the native test harness so
// the Teseo I2C stream behavior is tested without widening the public API.
#include "../../src/Hardware/drivers/TeseoGps.cpp"

namespace {
constexpr uint8_t kTestGpsAddress = 0x42;

void PrimeIdleProbe() {
    Wire.pushStreamByte(kTestGpsAddress, 0xFF);
}

bool SawRegisterWrite(uint8_t reg) {
    for (const auto& entry : Wire.writeLog()) {
        if (entry.address == kTestGpsAddress && entry.bytes.size() == 1 && entry.bytes[0] == reg) {
            return true;
        }
    }
    return false;
}
}  // namespace

void setUp() {
    Wire.clear();
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

void AssertEqualU32(uint32_t expected, uint32_t actual, const char* message) {
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

void test_initialize_uses_raw_stream_read_not_reserved_register() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize should succeed with idle stream byte");
    AssertTrue(!SawRegisterWrite(0xFD), "GPS must not address reserved 0xFD bytes-available register");
    AssertTrue(Wire.writeLog().empty(), "GPS stream init should not perform register-addressed writes");

    const TeseoGPS::Diagnostics diagnostics = gps.GetDiagnostics();
    AssertEqualU32(1, diagnostics.idleByteCount, "Idle probe byte should be counted");
}

void test_gga_sentence_updates_position_altitude_time_and_satellites() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");

    const Common::GPSData data = gps.ReadData();
    AssertFloatNear(48.1173f, data.latitude, 1e-4f, "GGA latitude mismatch");
    AssertFloatNear(11.516666f, data.longitude, 1e-4f, "GGA longitude mismatch");
    AssertFloatNear(545.4f, data.altitude, 1e-3f, "GGA altitude mismatch");
    AssertFloatNear(45319.0f, data.time, 1e-3f, "GGA time mismatch");
    AssertTrue(data.satellites == 8, "GGA satellites mismatch");
    AssertTrue(!SawRegisterWrite(0xFD), "GPS read must not address reserved 0xFD register");
}

void test_rmc_sentence_updates_position_when_split_across_reads() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");

    Wire.pushStreamBytes(kTestGpsAddress, "$GPRMC,123519,A,4807.038,N,01131.");
    Common::GPSData data = gps.ReadData();
    AssertFloatNear(0.0f, data.latitude, 1e-5f, "Partial RMC should not update latitude");

    Wire.pushStreamBytes(kTestGpsAddress, "000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    data = gps.ReadData();
    AssertFloatNear(48.1173f, data.latitude, 1e-4f, "Split RMC latitude mismatch");
    AssertFloatNear(11.516666f, data.longitude, 1e-4f, "Split RMC longitude mismatch");
    AssertFloatNear(45319.0f, data.time, 1e-3f, "Split RMC time mismatch");
}

void test_dummy_idle_bytes_do_not_corrupt_cached_data() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");
    const Common::GPSData first = gps.ReadData();

    Wire.pushStreamByte(kTestGpsAddress, 0xFF);
    Wire.pushStreamByte(kTestGpsAddress, 0xFF);
    const Common::GPSData second = gps.ReadData();

    AssertFloatNear(first.latitude, second.latitude, 1e-5f, "Idle bytes should keep cached latitude");
    AssertFloatNear(first.longitude, second.longitude, 1e-5f, "Idle bytes should keep cached longitude");
    AssertFloatNear(first.altitude, second.altitude, 1e-5f, "Idle bytes should keep cached altitude");
    AssertTrue(first.satellites == second.satellites, "Idle bytes should keep cached satellites");
}

void test_bad_checksum_is_rejected() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00\r\n");

    const Common::GPSData data = gps.ReadData();
    AssertFloatNear(0.0f, data.latitude, 1e-5f, "Bad checksum should not update latitude");
    AssertFloatNear(0.0f, data.longitude, 1e-5f, "Bad checksum should not update longitude");
    AssertTrue(data.satellites == 0, "Bad checksum should not update satellites");
}

void test_raw_read_failure_returns_stale_data_and_tracks_diagnostics() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    Wire.setRawReadFailure(kTestGpsAddress, true);

    const Common::GPSData data = gps.ReadData();
    const TeseoGPS::Diagnostics diagnostics = gps.GetDiagnostics();
    AssertFloatNear(0.0f, data.latitude, 1e-5f, "Read failure should return cached latitude");
    AssertEqualU32(1, diagnostics.readFailureCount, "Raw read failure should increment read failures");
    AssertEqualU32(1, diagnostics.staleReturnCount, "Raw read failure should increment stale returns");
}

int main(int, char**) {
    test_initialize_uses_raw_stream_read_not_reserved_register();
    test_gga_sentence_updates_position_altitude_time_and_satellites();
    test_rmc_sentence_updates_position_when_split_across_reads();
    test_dummy_idle_bytes_do_not_corrupt_cached_data();
    test_bad_checksum_is_rejected();
    test_raw_read_failure_returns_stale_data_and_tracks_diagnostics();
    std::cout << "PASS: Teseo GPS stream testbench\n";
    return 0;
}
