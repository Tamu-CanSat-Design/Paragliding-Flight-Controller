#include "interfaces/AudioBeaconInterface.h"

class AudioBeacon : public IAudioBeacon {
public:
    bool Initialize();
    void BuzzerOn();
    void BuzzerOff();
};

bool AudioBeacon::Initialize() {
    return true;
}

void AudioBeacon::BuzzerOn() {
}

void AudioBeacon::BuzzerOff() {
}
