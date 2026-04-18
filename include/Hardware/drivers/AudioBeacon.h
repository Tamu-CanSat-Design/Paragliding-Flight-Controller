#ifndef HARDWARE_AUDIO_BEACON_DRIVER_H
#define HARDWARE_AUDIO_BEACON_DRIVER_H

#include "../interfaces/AudioBeaconInterface.h"

class AudioBeacon : public IAudioBeacon {
public:
    bool Initialize() override;
    void BuzzerOn() override;
    void BuzzerOff() override;
};

#endif  // HARDWARE_AUDIO_BEACON_DRIVER_H
