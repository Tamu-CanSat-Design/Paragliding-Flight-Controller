#include "Hardware/drivers/Radio.h"

#include <Arduino.h>

namespace {
// XBee UART defaults:
// - Datasheet default is typically 9600 baud in command mode.
// - This project currently uses 115200 in Commlink, so we match that by default.
// Override at build time with -DXBEE_BAUDRATE=<value> if needed.
#ifndef XBEE_BAUDRATE
constexpr unsigned long kXbeeBaudRate = 115200UL;
#else
constexpr unsigned long kXbeeBaudRate = static_cast<unsigned long>(XBEE_BAUDRATE);
#endif

// XBee UART packet examples in the datasheet use 8-N-1 framing.
constexpr uint16_t kXbeeSerialConfig = SERIAL_8N1;

// Drain a bounded number of bytes on startup to clear stale noise/boot text.
constexpr size_t kStartupDrainLimit = 512;
}  // namespace

bool Radio::Initialize() {
    initialized_ = false;

    // Bring up the UART for the XBee serial interface.
    uart_.Begin(kXbeeBaudRate, kXbeeSerialConfig);

    // Allow UART and module startup transients to settle.
    delay(10);

    // Discard any residual bytes pending in RX so new traffic starts clean.
    size_t drained = 0;
    while (uart_.Available() > 0 && drained < kStartupDrainLimit) {
        (void)uart_.Read();
        ++drained;
    }

    initialized_ = true;
    return true;
}

bool Radio::Send(const uint8_t* data, size_t length) {
    if (!initialized_ || data == nullptr || length == 0) {
        return false;
    }

    // Raw transport: compatible with transparent mode and API frames.
    return uart_.Write(data, length) == length;
}

int Radio::Receive(uint8_t* buffer, size_t maxLength) {
    if (!initialized_ || buffer == nullptr || maxLength == 0) {
        return 0;
    }

    // Non-blocking read: consume only what is currently available.
    size_t count = 0;
    while (count < maxLength && uart_.Available() > 0) {
        const int byteValue = uart_.Read();
        if (byteValue < 0) {
            break;
        }
        buffer[count++] = static_cast<uint8_t>(byteValue);
    }

    return static_cast<int>(count);
}
