#ifndef HARDWARE_GPS_DRIVER_H
#define HARDWARE_GPS_DRIVER_H

#include <cstddef>
#include <cstdint>

#include "../bus/I2CDevice.h"
#include "../interfaces/GPSInterface.h"

class TeseoGPS : public IGPS {
public:
    static constexpr const char* kSensorName = "Generic NMEA GPS";

    TeseoGPS() = default;
    explicit TeseoGPS(TwoWire& wire);
    explicit TeseoGPS(uint8_t address);
    explicit TeseoGPS(I2CDevice& bus);
    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::GPSData ReadData() override;

private:
    static constexpr size_t kMaxSentenceLen = 128;
    static constexpr size_t kMaxFields = 20;
    static constexpr size_t kMaxI2cChunkLen = 32;

    I2CDevice i2c_{};
    I2CDevice* bus_ = &i2c_;
    Common::GPSData data_{};
    char sentence_[kMaxSentenceLen]{};
    size_t sentence_len_ = 0;
    bool initialized_ = false;

    void ProcessChar(char c);
    void HandleSentence(char* sentence);
    void ParseGga(char* const* fields, size_t count);
    void ParseRmc(char* const* fields, size_t count);
    static bool ParseDegrees(const char* field, bool is_latitude, float* out);
    static float ParseTimeSeconds(const char* field);
    static bool CheckChecksum(const char* start, const char* asterisk);
    static uint8_t HexToNibble(char c);
};

#endif  // HARDWARE_GPS_DRIVER_H
