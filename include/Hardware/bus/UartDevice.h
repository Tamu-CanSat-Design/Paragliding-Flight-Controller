#ifndef HARDWARE_BUS_UART_DEVICE_H
#define HARDWARE_BUS_UART_DEVICE_H

#include <cstddef>
#include <cstdint>

#include <Arduino.h>

#include "StreamBus.h"

// UART wrapper that plugs into the stream interface for GPS/modem style devices.
class UartDevice : public IStreamBus {
public:
    // Default to Serial1 because Serial is USB on Teensy and not HardwareSerial.
    explicit UartDevice(HardwareSerial& serial = Serial1)
        : serial_(&serial) {}

    // Serial begin stays here so drivers don't depend on HardwareSerial directly.
    void Begin(unsigned long baud, uint16_t config = SERIAL_8N1) const {
        // Configure the underlying hardware UART.
        serial_->begin(baud, config);
    }

    size_t Write(const uint8_t* data, size_t length) override {
        // Forward to HardwareSerial for buffered transmit.
        return serial_->write(data, length);
    }

    int Available() override {
        // Report how many bytes are ready in the RX buffer.
        return serial_->available();
    }

    int Read() override {
        // Read one byte from the RX buffer.
        return serial_->read();
    }

    size_t ReadBytes(uint8_t* buffer, size_t length) override {
        // Read up to length bytes (blocks until timeout if configured).
        return serial_->readBytes(buffer, length);
    }

private:
    HardwareSerial* const serial_;
};

#endif  // HARDWARE_BUS_UART_DEVICE_H
