#ifndef HARDWARE_GPS_DRIVER_H
#define HARDWARE_GPS_DRIVER_H

#include <cstddef>
#include <cstdint>

#include <MicroNMEA.h>

#include "../bus/I2CDevice.h"
#include "../interfaces/GPSInterface.h"

class TeseoGPS : public IGPS {
public:
    static constexpr const char* kSensorName = "Generic NMEA GPS";

    struct Diagnostics {
        uint32_t initFailureCount = 0;
        uint32_t readFailureCount = 0;
        uint32_t staleReturnCount = 0;
        uint32_t idleByteCount = 0;
        uint32_t invalidGroundSpeedCount = 0;
        uint32_t invalidCourseOverGroundCount = 0;
        uint32_t badChecksumCount = 0;
        uint32_t unknownSentenceCount = 0;
        uint32_t i2cReadAttemptCount = 0;
        uint32_t i2cReadSuccessCount = 0;
        uint32_t i2cBytesRequestedCount = 0;
        uint32_t i2cBytesReceivedCount = 0;
        uint32_t nonIdleByteCount = 0;
        uint32_t parserClearCount = 0;
        uint32_t sentenceCompleteCount = 0;
        uint32_t emptySentenceCount = 0;
        uint32_t ggaSentenceCount = 0;
        uint32_t rmcSentenceCount = 0;
        uint32_t ggaValidCount = 0;
        uint32_t rmcValidCount = 0;
        uint32_t ggaInvalidFixCount = 0;
        uint32_t rmcInvalidFixCount = 0;
        uint8_t lastI2cByte = 0;
        uint8_t lastNonIdleByte = 0;
        char lastMessageId[4] = {};
    };

    const char* SensorName() const override { return kSensorName; }
    TeseoGPS() = default;
    explicit TeseoGPS(uint8_t address);
    explicit TeseoGPS(I2CDevice& bus);
    bool Initialize() override;
    Common::GPSData ReadData() override;
    Diagnostics GetDiagnostics() const;
    void ClearDiagnostics();

private:
    static constexpr size_t kMaxI2cChunkLen = 32;
    static constexpr size_t kNmeaBufferLen = 128;

    I2CDevice i2c_{};
    I2CDevice* bus_ = &i2c_;
    Common::GPSData data_{};
    Diagnostics diagnostics_{};
    char nmea_buffer_[kNmeaBufferLen]{};
    MicroNMEA nmea_{nmea_buffer_, sizeof(nmea_buffer_)};
    bool initialized_ = false;

    void ProcessChar(char c);
    void CopyParsedSentence();
    void CopyPositionAndTime();
    void CopyGgaData();
    void CopyRmcData();
    bool RmcNumericFieldValid(uint8_t field_index, bool course_field) const;
};

#endif  // HARDWARE_GPS_DRIVER_H
