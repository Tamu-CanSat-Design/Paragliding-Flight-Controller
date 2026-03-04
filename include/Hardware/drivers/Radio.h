#ifndef HARDWARE_RADIO_DRIVER_H
#define HARDWARE_RADIO_DRIVER_H

#include "../bus/UartDevice.h"
#include "../interfaces/RadioInterface.h"

class Radio : public IRadio {
public:
    bool Initialize() override;
    bool Send(const uint8_t* data, size_t length) override;
    int Receive(uint8_t* buffer, size_t maxLength) override;

private:
    // UART transport to the XBee module (defaults to Serial1 in UartDevice).
    UartDevice uart_{};
    // Guards reads/writes before UART has been configured.
    bool initialized_ = false;
};

#endif  // HARDWARE_RADIO_DRIVER_H
