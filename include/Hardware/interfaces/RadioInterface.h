#ifndef HARDWARE_RADIO_H
#define HARDWARE_RADIO_H

#include <cstddef>
#include <cstdint>

class IRadio {
public:
    virtual ~IRadio() = default;
    virtual bool Initialize() = 0;
    virtual bool Send(const uint8_t* data, size_t length) = 0;
    virtual int Receive(uint8_t* buffer, size_t maxLength) = 0;
};

#endif  // HARDWARE_RADIO_H
