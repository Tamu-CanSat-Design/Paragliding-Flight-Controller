#ifndef HARDWARE_AUDIO_BEACON_H
#define HARDWARE_AUDIO_BEACON_H

class IAudioBeacon {
public:
    virtual ~IAudioBeacon() = default;
    virtual bool Initialize() = 0;
    virtual void BuzzerOn() = 0;
    virtual void BuzzerOff() = 0;
};

#endif  // HARDWARE_AUDIO_BEACON_H
