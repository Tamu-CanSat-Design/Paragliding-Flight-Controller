#include "drivers/TeseoGps.h"

#include <cstdlib>
#include <cstring>

#include <Wire.h>

namespace {
#ifndef GPS_I2C_ADDRESS
constexpr uint8_t kGpsI2cAddress = 0x42;
#else
constexpr uint8_t kGpsI2cAddress = static_cast<uint8_t>(GPS_I2C_ADDRESS);
#endif

#ifndef GPS_I2C_BYTES_AVAILABLE_REG
constexpr uint8_t kGpsBytesAvailableReg = 0xFD;
#else
constexpr uint8_t kGpsBytesAvailableReg = static_cast<uint8_t>(GPS_I2C_BYTES_AVAILABLE_REG);
#endif

#ifndef GPS_I2C_DATA_REG
constexpr uint8_t kGpsDataReg = 0xFF;
#else
constexpr uint8_t kGpsDataReg = static_cast<uint8_t>(GPS_I2C_DATA_REG);
#endif

#ifndef GPS_I2C_MAX_BYTES_PER_READ
constexpr size_t kGpsMaxBytesPerRead = 128;
#else
constexpr size_t kGpsMaxBytesPerRead = static_cast<size_t>(GPS_I2C_MAX_BYTES_PER_READ);
#endif
}  // namespace

GPS::GPS(uint8_t address)
    : i2c_(address), bus_(&i2c_) {}

GPS::GPS(I2CDevice& bus)
    : bus_(&bus) {}

bool GPS::Initialize() {
    data_ = {};
    sentence_len_ = 0;
    initialized_ = false;

    Wire.begin();
    Wire.setClock(Common::I2C_BAUDRATE);

    if (bus_ == &i2c_ && bus_->address() == 0) {
        bus_->setAddress(kGpsI2cAddress);
    }

    if (bus_->address() == 0) {
        return false;
    }

    uint8_t available_bytes[2] = {};
    if (!bus_->ReadRegisters(kGpsBytesAvailableReg, available_bytes, sizeof(available_bytes), false)) {
        return false;
    }

    initialized_ = true;
    return true;
}

Common::GPSData GPS::ReadData() {
    if (!initialized_) {
        return data_;
    }

    uint8_t available_bytes[2] = {};
    if (!bus_->ReadRegisters(kGpsBytesAvailableReg, available_bytes, sizeof(available_bytes), false)) {
        return data_;
    }

    size_t bytes_to_read = static_cast<size_t>(available_bytes[0])
        | (static_cast<size_t>(available_bytes[1]) << 8);
    if (bytes_to_read == 0) {
        return data_;
    }

    if (bytes_to_read > kGpsMaxBytesPerRead) {
        bytes_to_read = kGpsMaxBytesPerRead;
    }

    uint8_t chunk[kMaxI2cChunkLen] = {};
    while (bytes_to_read > 0) {
        size_t chunk_len = bytes_to_read;
        if (chunk_len > kMaxI2cChunkLen) {
            chunk_len = kMaxI2cChunkLen;
        }

        if (!bus_->ReadRegisters(kGpsDataReg, chunk, chunk_len, false)) {
            break;
        }

        for (size_t i = 0; i < chunk_len; ++i) {
            ProcessChar(static_cast<char>(chunk[i]));
        }
        bytes_to_read -= chunk_len;
    }

    return data_;
}
    
void GPS::ProcessChar(char c) {
    if (c == '$') {
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
        sentence_len_ = 0;
    }
}

void GPS::HandleSentence(char* sentence) {
    if (sentence[0] != '$') {
        return;
    }

    char* asterisk = std::strchr(sentence, '*');
    if (asterisk != nullptr) {
        if (!CheckChecksum(sentence + 1, asterisk)) {
            return;
        }
        *asterisk = '\0';
    }

    char* fields[kMaxFields] = {};
    size_t field_count = 0;

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

    const char* message = type + 2;
    if (std::strncmp(message, "GGA", 3) == 0) {
        ParseGga(fields, field_count);
    } else if (std::strncmp(message, "RMC", 3) == 0) {
        ParseRmc(fields, field_count);
    }
}

void GPS::ParseGga(char* const* fields, size_t count) {
    if (count < 10) {
        return;
    }

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

void GPS::ParseRmc(char* const* fields, size_t count) {
    if (count < 7) {
        return;
    }

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

    data_.time = ParseTimeSeconds(fields[1]);
    data_.latitude = latitude;
    data_.longitude = longitude;
}

bool GPS::ParseDegrees(const char* field, bool is_latitude, float* out) {
    if (field == nullptr || field[0] == '\0' || out == nullptr) {
        return false;
    }

    char* end = nullptr;
    const double raw = std::strtod(field, &end);
    if (end == field) {
        return false;
    }

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

float GPS::ParseTimeSeconds(const char* field) {
    if (field == nullptr || field[0] == '\0') {
        return 0.0f;
    }

    char* end = nullptr;
    const double raw = std::strtod(field, &end);
    if (end == field) {
        return 0.0f;
    }

    const int hours = static_cast<int>(raw / 10000.0);
    const int minutes = static_cast<int>((raw - (hours * 10000.0)) / 100.0);
    const double seconds = raw - (hours * 10000.0) - (minutes * 100.0);
    return static_cast<float>((hours * 3600.0) + (minutes * 60.0) + seconds);
}

bool GPS::CheckChecksum(const char* start, const char* asterisk) {
    if (start == nullptr || asterisk == nullptr || asterisk <= start) {
        return false;
    }

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

uint8_t GPS::HexToNibble(char c) {
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
