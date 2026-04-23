#ifndef HARDWARE_AUDIO_BEACON_DRIVER_H
#define HARDWARE_AUDIO_BEACON_DRIVER_H

#include <cstdint>

#include "../interfaces/AudioBeaconInterface.h"

class AudioBeacon : public IAudioBeacon {
public:
    struct Diagnostics {
        uint32_t initFailureCount = 0;
    };

    bool Initialize() override;
    void BuzzerOn() override;
    void BuzzerOff() override;
    Diagnostics GetDiagnostics() const;
    void ClearDiagnostics();

private:
    Diagnostics diagnostics_{};
    bool initialized_ = false;
};

#endif  // HARDWARE_AUDIO_BEACON_DRIVER_H
