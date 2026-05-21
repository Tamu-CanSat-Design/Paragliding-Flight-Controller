#include "Hardware/drivers/TeseoGPS.h"

#include <climits>
#include <cstdio>
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
constexpr size_t kGpsMaxBytesPerRead = 256;
#else
constexpr size_t kGpsMaxBytesPerRead = static_cast<size_t>(GPS_I2C_MAX_BYTES_PER_READ);
#endif

constexpr uint8_t kGpsIdleByte = 0xFF;
constexpr float kMillionthsToDegrees = 1.0e-6f;
constexpr float kMillimetersToMeters = 0.001f;
constexpr float kThousandthsToUnits = 0.001f;
constexpr float kKnotsToMetersPerSecond = 0.514444f;
constexpr float kStationarySpeedThresholdMetersPerSecond = 0.5f;

bool FieldIsStrictNumber(const char* start, const char* end, bool course_field) {
    if (start == nullptr || end == nullptr || start >= end) {
        return false;
    }

    char buffer[18] = {};
    const size_t len = static_cast<size_t>(end - start);
    if (len >= sizeof(buffer)) {
        return false;
    }

    for (size_t i = 0; i < len; ++i) {
        buffer[i] = start[i];
    }

    char* parse_end = nullptr;
    const double value = std::strtod(buffer, &parse_end);
    if (parse_end == buffer || *parse_end != '\0' || value < 0.0) {
        return false;
    }

    if (course_field && value >= 360.0) {
        return false;
    }

    return true;
}

void StoreMessageId(char (&destination)[4], const char* message_id) {
    destination[0] = '\0';
    destination[1] = '\0';
    destination[2] = '\0';
    destination[3] = '\0';
    if (message_id == nullptr) {
        return;
    }

    for (size_t i = 0; i < 3 && message_id[i] != '\0'; ++i) {
        destination[i] = message_id[i];
    }
}

void PrintInitDiagnostics(const char* stage, const TeseoGPS::Diagnostics& diagnostics, uint8_t address) {
    Serial.print("GPS init diag [");
    Serial.print(stage);
    Serial.print("]: addr=0x");
    Serial.print(address, HEX);
    Serial.print(" initFail=");
    Serial.print(diagnostics.initFailureCount);
    Serial.print(" attempts=");
    Serial.print(diagnostics.i2cReadAttemptCount);
    Serial.print(" success=");
    Serial.print(diagnostics.i2cReadSuccessCount);
    Serial.print(" bytesReq=");
    Serial.print(diagnostics.i2cBytesRequestedCount);
    Serial.print(" bytesRx=");
    Serial.print(diagnostics.i2cBytesReceivedCount);
    Serial.print(" idle=");
    Serial.print(diagnostics.idleByteCount);
    Serial.print(" data=");
    Serial.print(diagnostics.nonIdleByteCount);
    Serial.print(" last=0x");
    Serial.print(diagnostics.lastI2cByte, HEX);
    Serial.print(" lastData=0x");
    Serial.print(diagnostics.lastNonIdleByte, HEX);
    Serial.println();
}
}  // namespace

TeseoGPS::TeseoGPS(uint8_t address)
    : i2c_(address), bus_(&i2c_) {}

TeseoGPS::TeseoGPS(I2CDevice& bus)
    : bus_(&bus) {}

bool TeseoGPS::Initialize() {
    data_ = {};
    diagnostics_ = {};
    nmea_.clear();
    initialized_ = false;
    PrintInitDiagnostics("start", diagnostics_, bus_->address());

    Wire.begin();
    Wire.setClock(Common::I2C_BAUDRATE);
    PrintInitDiagnostics("wire-ready", diagnostics_, bus_->address());
    delay(1000);
    if (bus_ == &i2c_ && bus_->address() == 0) {
        bus_->setAddress(kGpsI2cAddress);
    }
    PrintInitDiagnostics("address-set", diagnostics_, bus_->address());
    delay(1000);
    if (bus_->address() == 0) {
        ++diagnostics_.initFailureCount;
        PrintInitDiagnostics("fail-zero-address", diagnostics_, bus_->address());
        return false;
    }
    delay(2000);

    uint8_t probe = kGpsIdleByte;
    ++diagnostics_.i2cReadAttemptCount;
    ++diagnostics_.i2cBytesRequestedCount;
    PrintInitDiagnostics("before-probe", diagnostics_, bus_->address());
    if (!bus_->ReadBytes(&probe, 1)) {
        ++diagnostics_.initFailureCount;
        PrintInitDiagnostics("probe-read-failed", diagnostics_, bus_->address());
        return false;
    }
    ++diagnostics_.i2cReadSuccessCount;
    ++diagnostics_.i2cBytesReceivedCount;
    diagnostics_.lastI2cByte = probe;
    PrintInitDiagnostics("probe-read-ok", diagnostics_, bus_->address());
    delay(2000);
    if (probe != kGpsIdleByte) {
        ++diagnostics_.nonIdleByteCount;
        diagnostics_.lastNonIdleByte = probe;
        ProcessChar(static_cast<char>(probe));
        PrintInitDiagnostics("probe-data-byte", diagnostics_, bus_->address());
    } else {
        ++diagnostics_.idleByteCount;
        ++diagnostics_.parserClearCount;
        nmea_.clear();
        PrintInitDiagnostics("probe-idle-byte", diagnostics_, bus_->address());
    }
    initialized_ = true;
    PrintInitDiagnostics("complete", diagnostics_, bus_->address());
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

        ++diagnostics_.i2cReadAttemptCount;
        diagnostics_.i2cBytesRequestedCount += static_cast<uint32_t>(chunk_len);
        if (!bus_->ReadBytes(chunk, chunk_len)) {
            ++diagnostics_.readFailureCount;
            ++diagnostics_.staleReturnCount;
            break;
        }
        ++diagnostics_.i2cReadSuccessCount;
        diagnostics_.i2cBytesReceivedCount += static_cast<uint32_t>(chunk_len);

        bool saw_idle_byte = false;
        for (size_t i = 0; i < chunk_len; ++i) {
            diagnostics_.lastI2cByte = chunk[i];
            if (chunk[i] == kGpsIdleByte) {
                ++diagnostics_.idleByteCount;
                saw_idle_byte = true;
                ++diagnostics_.parserClearCount;
                nmea_.clear();
                continue;
            }
            ++diagnostics_.nonIdleByteCount;
            diagnostics_.lastNonIdleByte = chunk[i];
            ProcessChar(static_cast<char>(chunk[i]));
        }

        bytes_to_read -= chunk_len;
        if (saw_idle_byte) {
            break;
        }
    }

    return data_;
}

void TeseoGPS::ProcessChar(char c) {
    if (nmea_.process(c)) {
        ++diagnostics_.sentenceCompleteCount;
        CopyParsedSentence();
    }
}

void TeseoGPS::CopyParsedSentence() {
    const char* sentence = nmea_.getSentence();
    if (sentence == nullptr || sentence[0] == '\0') {
        ++diagnostics_.emptySentenceCount;
        return;
    }

    const char* message_id = nmea_.getMessageID();
    StoreMessageId(diagnostics_.lastMessageId, message_id);
    if (message_id == nullptr) {
        ++diagnostics_.unknownSentenceCount;
        return;
    }

    if (std::strcmp(message_id, "GGA") == 0) {
        ++diagnostics_.ggaSentenceCount;
        CopyGgaData();
    } else if (std::strcmp(message_id, "RMC") == 0) {
        ++diagnostics_.rmcSentenceCount;
        CopyRmcData();
    } else {
        ++diagnostics_.unknownSentenceCount;
    }
}

void TeseoGPS::CopyPositionAndTime() {
    std::snprintf(data_.time,
                  sizeof(data_.time),
                  "%02u:%02u:%02u",
                  static_cast<unsigned>(nmea_.getHour()),
                  static_cast<unsigned>(nmea_.getMinute()),
                  static_cast<unsigned>(nmea_.getSecond()));
    data_.latitude = static_cast<float>(nmea_.getLatitude()) * kMillionthsToDegrees;
    data_.longitude = static_cast<float>(nmea_.getLongitude()) * kMillionthsToDegrees;
}

void TeseoGPS::CopyGgaData() {
    if (!nmea_.isValid()) {
        ++diagnostics_.ggaInvalidFixCount;
        return;
    }

    ++diagnostics_.ggaValidCount;
    CopyPositionAndTime();
    data_.satellites = static_cast<int>(nmea_.getNumSatellites());

    long altitude_mm = 0;
    if (nmea_.getAltitude(altitude_mm)) {
        data_.altitude = static_cast<float>(altitude_mm) * kMillimetersToMeters;
    }
}

void TeseoGPS::CopyRmcData() {
    if (!nmea_.isValid()) {
        ++diagnostics_.rmcInvalidFixCount;
        data_.groundSpeedMetersPerSecond = 0.0f;
        data_.courseOverGroundDegrees = 0.0f;
        data_.groundSpeedValid = false;
        data_.courseOverGroundValid = false;
        ++diagnostics_.invalidGroundSpeedCount;
        ++diagnostics_.invalidCourseOverGroundCount;
        return;
    }

    ++diagnostics_.rmcValidCount;
    CopyPositionAndTime();

    const bool speed_valid = RmcNumericFieldValid(7, false);
    if (speed_valid && nmea_.getSpeed() != LONG_MIN) {
        const float ground_speed_mps =
            static_cast<float>(nmea_.getSpeed()) * kThousandthsToUnits * kKnotsToMetersPerSecond;
        if (ground_speed_mps < kStationarySpeedThresholdMetersPerSecond) {
            data_.groundSpeedMetersPerSecond = 0.0f;
            data_.courseOverGroundDegrees = 0.0f;
            data_.groundSpeedValid = true;
            data_.courseOverGroundValid = true;
            return;
        }

        data_.groundSpeedMetersPerSecond = ground_speed_mps;
        data_.groundSpeedValid = true;
    } else {
        data_.groundSpeedMetersPerSecond = 0.0f;
        data_.courseOverGroundDegrees = 0.0f;
        data_.groundSpeedValid = false;
        data_.courseOverGroundValid = false;
        ++diagnostics_.invalidGroundSpeedCount;
        ++diagnostics_.invalidCourseOverGroundCount;
        return;
    }

    const bool course_valid = RmcNumericFieldValid(8, true);
    if (course_valid && nmea_.getCourse() != LONG_MIN) {
        data_.courseOverGroundDegrees = static_cast<float>(nmea_.getCourse()) * kThousandthsToUnits;
        data_.courseOverGroundValid = true;
    } else {
        data_.courseOverGroundDegrees = 0.0f;
        data_.courseOverGroundValid = false;
        ++diagnostics_.invalidCourseOverGroundCount;
    }
}

bool TeseoGPS::RmcNumericFieldValid(uint8_t field_index, bool course_field) const {
    const char* sentence = nmea_.getSentence();
    if (sentence == nullptr || sentence[0] == '\0') {
        return false;
    }

    const char* field_start = sentence;
    if (*field_start == '$') {
        ++field_start;
    }

    uint8_t current_field = 0;
    const char* cursor = field_start;
    while (*cursor != '\0' && *cursor != '*' && *cursor != '\r' && *cursor != '\n') {
        if (current_field == field_index) {
            const char* field_end = cursor;
            while (*field_end != '\0' && *field_end != ',' && *field_end != '*'
                   && *field_end != '\r' && *field_end != '\n') {
                ++field_end;
            }
            return FieldIsStrictNumber(cursor, field_end, course_field);
        }

        if (*cursor == ',') {
            ++current_field;
            field_start = cursor + 1;
        }
        ++cursor;
    }

    if (current_field == field_index) {
        return FieldIsStrictNumber(field_start, cursor, course_field);
    }

    return false;
}

TeseoGPS::Diagnostics TeseoGPS::GetDiagnostics() const {
    return diagnostics_;
}

void TeseoGPS::ClearDiagnostics() {
    diagnostics_ = {};
}
