#include "Hardware/drivers/TeseoGPS.h"

#include <cstdlib>
#include <cstring>

#include <Wire.h>

namespace {
#ifndef GPS_I2C_ADDRESS
constexpr uint8_t kGpsI2cAddress = 0x3A;
#else
constexpr uint8_t kGpsI2cAddress = static_cast<uint8_t>(GPS_I2C_ADDRESS);
#endif

#ifndef GPS_I2C_MAX_BYTES_PER_READ
constexpr size_t kGpsMaxBytesPerRead = 128;
#else
constexpr size_t kGpsMaxBytesPerRead = static_cast<size_t>(GPS_I2C_MAX_BYTES_PER_READ);
#endif

// the teseo streams 0xFF on the i2c bus when no nmea data is ready (st um2229)
constexpr uint8_t kGpsIdleByte = 0xFF;
// 1 knot = 0.514444 m/s (exact per international definition)
constexpr float kKnotsToMetersPerSecond = 0.514444f;
}  // namespace

TeseoGPS::TeseoGPS(uint8_t address)
    : i2c_(address), bus_(&i2c_) {}

TeseoGPS::TeseoGPS(I2CDevice& bus)
    : bus_(&bus) {}

bool TeseoGPS::Initialize() {
    data_ = {};
    diagnostics_ = {};
    sentence_len_ = 0;
    initialized_ = false;

    Wire.begin();
    Wire.setClock(Common::I2C_BAUDRATE);
    Serial.println("TeseoGPS: I2C bus initialized for GPS");
    if (bus_ == &i2c_ && bus_->address() == 0) {
        bus_->setAddress(kGpsI2cAddress);
    }
    Serial.println("TeseoGPS: I2C device address set to 0x" + String(bus_->address(), HEX));
    if (bus_->address() == 0) {
        ++diagnostics_.initFailureCount;
        return false;
    }
    Serial.println("TeseoGPS: Starting GPS initialization sequence");
    // st um2229 dm00398983: teseo i2c nmea reads are direct stream reads; 0xFF is the stream, 0x00-0xFE are reserved
    delay(2000);
    // probe one byte to confirm the bus is responsive before marking initialized
    uint8_t probe = kGpsIdleByte;
    if (!bus_->ReadBytes(&probe, 1)) {
        ++diagnostics_.initFailureCount;
        return false;
    }
    delay(2000);
    Serial.println("TeseoGPS: I2C probe read byte 0x" + String(probe, HEX));
    if (probe != kGpsIdleByte) {
        ProcessChar(static_cast<char>(probe));
    } else {
        ++diagnostics_.idleByteCount;
    }
    Serial.println("TeseoGPS: GPS initialization sequence complete");
    initialized_ = true;
    return true;
}

Common::GPSData TeseoGPS::ReadData() {
    if (!initialized_) {
        ++diagnostics_.staleReturnCount;
        return data_;
    }

    uint8_t chunk[kMaxI2cChunkLen] = {};
    size_t bytes_to_read = kGpsMaxBytesPerRead;
    while (bytes_to_read > 0) {
        size_t chunk_len = bytes_to_read;
        if (chunk_len > kMaxI2cChunkLen) {
            chunk_len = kMaxI2cChunkLen;
        }

        if (!bus_->ReadBytes(chunk, chunk_len)) {
            ++diagnostics_.readFailureCount;
            ++diagnostics_.staleReturnCount;
            break;
        }

        bool saw_idle_byte = false;
        for (size_t i = 0; i < chunk_len; ++i) {
            if (chunk[i] == kGpsIdleByte) {
                ++diagnostics_.idleByteCount;
                saw_idle_byte = true;
                continue;
            }
            ProcessChar(static_cast<char>(chunk[i]));
        }

        bytes_to_read -= chunk_len;
        // an idle byte means the stream is drained; stop reading early
        if (saw_idle_byte) {
            break;
        }
    }

    return data_;
}

void TeseoGPS::ProcessChar(char c) {
    if (c == '$') {
        // '$' always starts a fresh sentence, discarding any partial buffer
        sentence_len_ = 0;
        sentence_[sentence_len_++] = c;
        return;
    }

    if (sentence_len_ == 0) {
        return;
    }

    if (c == '\n') {
        sentence_[sentence_len_] = '\0';
        HandleSentence(sentence_);
        sentence_len_ = 0;
        return;
    }

    if (c == '\r') {
        return;
    }

    if (sentence_len_ < (kMaxSentenceLen - 1)) {
        sentence_[sentence_len_++] = c;
    } else {
        // sentence exceeded buffer; discard and wait for the next '$'
        sentence_len_ = 0;
    }
}

void TeseoGPS::HandleSentence(char* sentence) {
    if (sentence[0] != '$') {
        return;
    }

    char* asterisk = std::strchr(sentence, '*');
    if (asterisk != nullptr) {
        if (!CheckChecksum(sentence + 1, asterisk)) {
            return;
        }
        // null-terminate at '*' so field parsing never reaches checksum bytes
        *asterisk = '\0';
    }

    char* fields[kMaxFields] = {};
    size_t field_count = 0;

    // split on commas in-place; fields[] point directly into sentence buffer
    fields[field_count++] = sentence + 1;
    for (char* p = sentence + 1; *p != '\0' && field_count < kMaxFields; ++p) {
        if (*p == ',') {
            *p = '\0';
            fields[field_count++] = p + 1;
        }
    }

    if (field_count == 0) {
        return;
    }

    const char* type = fields[0];
    if (std::strlen(type) < 5) {
        return;
    }

    // skip the two-char talker id (e.g. "GP", "GN") to get the message type
    const char* message = type + 2;
    if (std::strncmp(message, "GGA", 3) == 0) {
        ParseGga(fields, field_count);
    } else if (std::strncmp(message, "RMC", 3) == 0) {
        ParseRmc(fields, field_count);
    }
}

void TeseoGPS::ParseGga(char* const* fields, size_t count) {
    if (count < 10) {
        return;
    }

    // fix quality 0 means no fix; negative values are malformed
    const int fix_quality = std::atoi(fields[6]);
    if (fix_quality <= 0) {
        return;
    }

    float latitude = 0.0f;
    float longitude = 0.0f;
    if (!ParseDegrees(fields[2], true, &latitude)) {
        return;
    }
    if (!ParseDegrees(fields[4], false, &longitude)) {
        return;
    }

    if (fields[3][0] == 'S') {
        latitude = -latitude;
    } else if (fields[3][0] != 'N') {
        return;
    }

    if (fields[5][0] == 'W') {
        longitude = -longitude;
    } else if (fields[5][0] != 'E') {
        return;
    }

    data_.time = ParseTimeSeconds(fields[1]);
    data_.latitude = latitude;
    data_.longitude = longitude;
    data_.satellites = std::atoi(fields[7]);

    if (fields[9][0] != '\0') {
        data_.altitude = static_cast<float>(std::atof(fields[9]));
    }
}

void TeseoGPS::ParseRmc(char* const* fields, size_t count) {
    // rmc field layout (0-indexed): type,time,status,lat,N/S,lon,E/W,speed_knots,cog_degrees,...
    if (count < 7) {
        return;
    }

    // status 'A' = active fix; anything else (typically 'V') means no valid data
    if (fields[2][0] != 'A') {
        return;
    }

    float latitude = 0.0f;
    float longitude = 0.0f;
    if (!ParseDegrees(fields[3], true, &latitude)) {
        return;
    }
    if (!ParseDegrees(fields[5], false, &longitude)) {
        return;
    }

    if (fields[4][0] == 'S') {
        latitude = -latitude;
    } else if (fields[4][0] != 'N') {
        return;
    }

    if (fields[6][0] == 'W') {
        longitude = -longitude;
    } else if (fields[6][0] != 'E') {
        return;
    }

    // keep the cached speed on parse failure so callers see last-good value
    float ground_speed_mps = data_.groundSpeedMetersPerSecond;
    bool ground_speed_valid = false;
    if (count >= 8 && ParseGroundSpeedMetersPerSecond(fields[7], &ground_speed_mps)) {
        ground_speed_valid = true;
    } else {
        ++diagnostics_.invalidGroundSpeedCount;
    }

    // cog is undefined when stationary; empty field is normal and not a fault
    float cog_degrees = data_.courseOverGroundDegrees;
    bool cog_valid = false;
    if (count >= 9 && ParseCourseOverGroundDegrees(fields[8], &cog_degrees)) {
        cog_valid = true;
    } else {
        ++diagnostics_.invalidCourseOverGroundCount;
    }

    data_.time = ParseTimeSeconds(fields[1]);
    data_.latitude = latitude;
    data_.longitude = longitude;
    data_.groundSpeedMetersPerSecond = ground_speed_mps;
    data_.groundSpeedValid = ground_speed_valid;
    data_.courseOverGroundDegrees = cog_degrees;
    data_.courseOverGroundValid = cog_valid;
}

bool TeseoGPS::ParseDegrees(const char* field, bool is_latitude, float* out) {
    if (field == nullptr || field[0] == '\0' || out == nullptr) {
        return false;
    }

    char* end = nullptr;
    const double raw = std::strtod(field, &end);
    if (end == field) {
        return false;
    }

    // nmea encodes position as DDDMM.MMMM; split into integer degrees and minutes
    const int degrees = static_cast<int>(raw / 100.0);
    const double minutes = raw - (degrees * 100.0);
    if (minutes < 0.0 || minutes >= 60.0) {
        return false;
    }

    const double value = degrees + (minutes / 60.0);
    const double max_degrees = is_latitude ? 90.0 : 180.0;
    if (degrees < 0 || value > max_degrees) {
        return false;
    }

    *out = static_cast<float>(value);
    return true;
}

bool TeseoGPS::ParseCourseOverGroundDegrees(const char* field, float* out) {
    if (field == nullptr || field[0] == '\0' || out == nullptr) {
        return false;
    }

    char* end = nullptr;
    const double cog = std::strtod(field, &end);
    // *end != '\0' catches trailing garbage; >= 360 rejects wrap-around (360 == 0)
    if (end == field || *end != '\0' || cog < 0.0 || cog >= 360.0) {
        return false;
    }

    *out = static_cast<float>(cog);
    return true;
}

bool TeseoGPS::ParseGroundSpeedMetersPerSecond(const char* field, float* out) {
    if (field == nullptr || field[0] == '\0' || out == nullptr) {
        return false;
    }

    char* end = nullptr;
    const double speed_knots = std::strtod(field, &end);
    if (end == field || *end != '\0' || speed_knots < 0.0) {
        return false;
    }

    *out = static_cast<float>(speed_knots) * kKnotsToMetersPerSecond;
    return true;
}

float TeseoGPS::ParseTimeSeconds(const char* field) {
    if (field == nullptr || field[0] == '\0') {
        return 0.0f;
    }

    char* end = nullptr;
    const double raw = std::strtod(field, &end);
    if (end == field) {
        return 0.0f;
    }

    // nmea time is HHMMSS.SS packed as a single decimal number
    const int hours = static_cast<int>(raw / 10000.0);
    const int minutes = static_cast<int>((raw - (hours * 10000.0)) / 100.0);
    const double seconds = raw - (hours * 10000.0) - (minutes * 100.0);
    return static_cast<float>((hours * 3600.0) + (minutes * 60.0) + seconds);
}

bool TeseoGPS::CheckChecksum(const char* start, const char* asterisk) {
    if (start == nullptr || asterisk == nullptr || asterisk <= start) {
        return false;
    }

    // nmea checksum is xor of all bytes between '$' and '*'
    uint8_t checksum = 0;
    for (const char* p = start; p < asterisk; ++p) {
        checksum ^= static_cast<uint8_t>(*p);
    }

    if (asterisk[1] == '\0' || asterisk[2] == '\0') {
        return false;
    }

    const uint8_t expected = static_cast<uint8_t>((HexToNibble(asterisk[1]) << 4) |
                                                   HexToNibble(asterisk[2]));
    return checksum == expected;
}

uint8_t TeseoGPS::HexToNibble(char c) {
    if (c >= '0' && c <= '9') {
        return static_cast<uint8_t>(c - '0');
    }
    if (c >= 'A' && c <= 'F') {
        return static_cast<uint8_t>(c - 'A' + 10);
    }
    if (c >= 'a' && c <= 'f') {
        return static_cast<uint8_t>(c - 'a' + 10);
    }
    return 0;
}

TeseoGPS::Diagnostics TeseoGPS::GetDiagnostics() const {
    return diagnostics_;
}

void TeseoGPS::ClearDiagnostics() {
    diagnostics_ = {};
}
