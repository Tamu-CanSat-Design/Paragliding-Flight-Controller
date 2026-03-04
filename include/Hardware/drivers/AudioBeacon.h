#ifndef HARDWARE_AUDIO_BEACON_DRIVER_H
#define HARDWARE_AUDIO_BEACON_DRIVER_H

#include <cstdint>

#include "../interfaces/AudioBeaconInterface.h"

class AudioBeacon : public IAudioBeacon {
public:
    // Default constructor keeps global/static allocation straightforward.
    AudioBeacon() = default;

    // Assigns the digital output pin used to switch the buzzer driver stage.
    // Must be called before Initialize() when using default construction.
    void ConfigurePin(uint8_t pin);

    // Selects output polarity:
    // - true:  pin HIGH turns buzzer on
    // - false: pin LOW turns buzzer on (active-low transistor stage)
    void ConfigureActiveLevel(bool activeHigh);

    bool Initialize() override;
    void BuzzerOn() override;
    void BuzzerOff() override;

private:
    static constexpr uint8_t kInvalidPin = 0xFF;

    void ApplyOutput(bool on);

    uint8_t pin_ = kInvalidPin;
    bool activeHigh_ = true;
    bool initialized_ = false;
    // Cached requested state so pre-init commands are applied after Initialize().
    bool requestedOn_ = false;
};

#endif  // HARDWARE_AUDIO_BEACON_DRIVER_H
