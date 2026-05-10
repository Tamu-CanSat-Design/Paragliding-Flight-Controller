#include <cmath>
#include <cstdlib>
#include <iostream>

#include "../support/Wire.h"
#include "Hardware/drivers/TeseoGPS.h"

// Pull the production implementation directly into the native test harness so
// the Teseo I2C stream behavior is tested without widening the public API.
#include "../../src/Hardware/drivers/TeseoGps.cpp"

namespace {
constexpr uint8_t kTestGpsAddress = 0x3A;

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
    AssertFloatNear(11.523546f, data.groundSpeedMetersPerSecond, 1e-5f, "Split RMC ground speed mismatch");
    AssertTrue(data.groundSpeedValid, "Split RMC ground speed should be valid");
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

void test_bad_rmc_speed_marks_degraded_without_overwriting_cached_speed() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    Common::GPSData data = gps.ReadData();
    AssertFloatNear(11.523546f, data.groundSpeedMetersPerSecond, 1e-5f, "Initial RMC ground speed mismatch");
    AssertTrue(data.groundSpeedValid, "Initial RMC ground speed should be valid");

    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,abc,084.4,230394,003.1,W*20\r\n");
    data = gps.ReadData();

    const TeseoGPS::Diagnostics diagnostics = gps.GetDiagnostics();
    AssertFloatNear(11.523546f, data.groundSpeedMetersPerSecond, 1e-5f,
                    "Bad RMC speed should leave cached speed value");
    AssertTrue(!data.groundSpeedValid, "Bad RMC speed should mark ground speed invalid");
    AssertEqualU32(1, diagnostics.invalidGroundSpeedCount,
                   "Bad RMC speed should increment invalid speed diagnostics");
}

void test_empty_rmc_speed_marks_degraded_without_overwriting_cached_speed() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    Common::GPSData data = gps.ReadData();
    AssertTrue(data.groundSpeedValid, "Initial RMC ground speed should be valid");

    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,,084.4,230394,003.1,W*40\r\n");
    data = gps.ReadData();

    const TeseoGPS::Diagnostics diagnostics = gps.GetDiagnostics();
    AssertFloatNear(11.523546f, data.groundSpeedMetersPerSecond, 1e-5f,
                    "Empty RMC speed should leave cached speed value");
    AssertTrue(!data.groundSpeedValid, "Empty RMC speed should mark ground speed invalid");
    AssertEqualU32(1, diagnostics.invalidGroundSpeedCount,
                   "Empty RMC speed should increment invalid speed diagnostics");
}

void test_inactive_rmc_does_not_update_cached_speed() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    const Common::GPSData first = gps.ReadData();

    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*7D\r\n");
    const Common::GPSData second = gps.ReadData();

    AssertFloatNear(first.groundSpeedMetersPerSecond, second.groundSpeedMetersPerSecond, 1e-5f,
                    "Inactive RMC should not update cached speed value");
    AssertTrue(second.groundSpeedValid, "Inactive RMC should not change cached speed validity");
}

void test_rmc_sentence_parses_course_over_ground() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    const Common::GPSData data = gps.ReadData();
    AssertFloatNear(84.4f, data.courseOverGroundDegrees, 1e-4f, "RMC COG mismatch");
    AssertTrue(data.courseOverGroundValid, "RMC COG should be valid");
}

void test_rmc_cog_north_boundary() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    // COG 000.0 (due north): checksum 0x6A ^ (0x30^0x38^0x34^0x2E^0x34) ^ 0 = 0x6A ^ 0x26 ^ 0 (same length, replacing 084.4 with 000.0: diff 0x08) = 0x62
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,000.0,230394,003.1,W*62\r\n");
    const Common::GPSData data = gps.ReadData();
    AssertFloatNear(0.0f, data.courseOverGroundDegrees, 1e-4f, "RMC COG 000.0 mismatch");
    AssertTrue(data.courseOverGroundValid, "RMC COG 000.0 should be valid");
}

void test_rmc_cog_upper_boundary() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    // COG 359.9: diff from 084.4 = 0x03^0x0D^0x0D^0x0D = 0x0E, checksum = 0x6A^0x0E = 0x64
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,359.9,230394,003.1,W*64\r\n");
    const Common::GPSData data = gps.ReadData();
    AssertFloatNear(359.9f, data.courseOverGroundDegrees, 1e-3f, "RMC COG 359.9 mismatch");
    AssertTrue(data.courseOverGroundValid, "RMC COG 359.9 should be valid");
}

void test_rmc_cog_360_is_rejected() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    // COG 360.0 is out of range and must be rejected.
    // diff from 084.4: 0x03^0x0E^0x04^0x04 = 0x0D, checksum = 0x6A^0x0D = 0x67
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,360.0,230394,003.1,W*67\r\n");
    const Common::GPSData data = gps.ReadData();
    AssertTrue(!data.courseOverGroundValid, "RMC COG 360.0 should be rejected as invalid");
    const TeseoGPS::Diagnostics diagnostics = gps.GetDiagnostics();
    AssertEqualU32(1, diagnostics.invalidCourseOverGroundCount,
                   "Out-of-range COG should increment invalid COG diagnostics");
}

void test_empty_rmc_cog_marks_degraded_without_overwriting_cached_cog() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    Common::GPSData data = gps.ReadData();
    AssertTrue(data.courseOverGroundValid, "Initial RMC COG should be valid");

    // Empty COG field (stationary receiver). Remove 084.4 (XOR = 0x26), checksum = 0x6A^0x26 = 0x4C
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,,230394,003.1,W*4C\r\n");
    data = gps.ReadData();
    const TeseoGPS::Diagnostics diagnostics = gps.GetDiagnostics();
    AssertFloatNear(84.4f, data.courseOverGroundDegrees, 1e-4f,
                    "Empty RMC COG should leave cached value unchanged");
    AssertTrue(!data.courseOverGroundValid, "Empty RMC COG should mark COG invalid");
    AssertEqualU32(1, diagnostics.invalidCourseOverGroundCount,
                   "Empty RMC COG should increment invalid COG diagnostics");
}

void test_bad_rmc_cog_marks_degraded_without_overwriting_cached_cog() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    Common::GPSData data = gps.ReadData();
    AssertTrue(data.courseOverGroundValid, "Initial RMC COG should be valid");

    // Non-numeric COG field. Replace 084.4 with abc: diff 0x26^0x60=0x46, checksum=0x6A^0x46=0x2C
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,abc,230394,003.1,W*2C\r\n");
    data = gps.ReadData();
    const TeseoGPS::Diagnostics diagnostics = gps.GetDiagnostics();
    AssertFloatNear(84.4f, data.courseOverGroundDegrees, 1e-4f,
                    "Bad RMC COG should leave cached value unchanged");
    AssertTrue(!data.courseOverGroundValid, "Bad RMC COG should mark COG invalid");
    AssertEqualU32(1, diagnostics.invalidCourseOverGroundCount,
                   "Bad RMC COG should increment invalid COG diagnostics");
}

void test_rmc_with_too_few_fields_marks_cog_invalid() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    // Sentence ends after field 7 (speed); no COG field present. Checksum = 0x38.
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4*38\r\n");
    const Common::GPSData data = gps.ReadData();
    AssertTrue(!data.courseOverGroundValid, "RMC with too few fields should mark COG invalid");
    const TeseoGPS::Diagnostics diagnostics = gps.GetDiagnostics();
    AssertEqualU32(1, diagnostics.invalidCourseOverGroundCount,
                   "Too-few-fields RMC should increment invalid COG diagnostics");
}

void test_inactive_rmc_does_not_update_cached_cog() {
    setUp();
    PrimeIdleProbe();

    TeseoGPS gps;
    AssertTrue(gps.Initialize(), "GPS initialize failed");
    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    const Common::GPSData first = gps.ReadData();

    Wire.pushStreamBytes(kTestGpsAddress,
                         "$GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*7D\r\n");
    const Common::GPSData second = gps.ReadData();

    AssertFloatNear(first.courseOverGroundDegrees, second.courseOverGroundDegrees, 1e-5f,
                    "Inactive RMC should not update cached COG value");
    AssertTrue(second.courseOverGroundValid, "Inactive RMC should not change cached COG validity");
}

int main(int, char**) {
    test_initialize_uses_raw_stream_read_not_reserved_register();
    test_gga_sentence_updates_position_altitude_time_and_satellites();
    test_rmc_sentence_updates_position_when_split_across_reads();
    test_dummy_idle_bytes_do_not_corrupt_cached_data();
    test_bad_checksum_is_rejected();
    test_raw_read_failure_returns_stale_data_and_tracks_diagnostics();
    test_bad_rmc_speed_marks_degraded_without_overwriting_cached_speed();
    test_empty_rmc_speed_marks_degraded_without_overwriting_cached_speed();
    test_inactive_rmc_does_not_update_cached_speed();
    test_rmc_sentence_parses_course_over_ground();
    test_rmc_cog_north_boundary();
    test_rmc_cog_upper_boundary();
    test_rmc_cog_360_is_rejected();
    test_empty_rmc_cog_marks_degraded_without_overwriting_cached_cog();
    test_bad_rmc_cog_marks_degraded_without_overwriting_cached_cog();
    test_rmc_with_too_few_fields_marks_cog_invalid();
    test_inactive_rmc_does_not_update_cached_cog();
    std::cout << "PASS: Teseo GPS stream testbench\n";
    return 0;
}
