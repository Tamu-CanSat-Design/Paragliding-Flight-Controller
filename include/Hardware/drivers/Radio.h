#ifndef HARDWARE_RADIO_DRIVER_H
#define HARDWARE_RADIO_DRIVER_H

#include "../interfaces/RadioInterface.h"

class Radio : public IRadio {
public:
    bool Initialize() override;
    bool Send(const uint8_t* data, size_t length) override;
    int Receive(uint8_t* buffer, size_t maxLength) override;
};

#endif  // HARDWARE_RADIO_DRIVER_H
