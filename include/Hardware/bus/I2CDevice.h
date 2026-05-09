#ifndef HARDWARE_BUS_I2C_DEVICE_H
#define HARDWARE_BUS_I2C_DEVICE_H

#include <cstddef>
#include <cstdint>

#include "Wire.h"

#include "RegisterBus.h"

// Thin wrapper around TwoWire so drivers keep register logic in one place.
class I2CDevice : public IRegisterBus {
public:
    // Store the address and wire instance so multiple devices can share a bus.
    explicit I2CDevice(uint8_t address = 0, TwoWire& wire = Wire)
        : address_(address), wire_(&wire) {}

    // Update the target I2C address (useful for probing alt addresses).
    void setAddress(uint8_t address) { address_ = address; }
    // Expose the current address for logging or debug.
    uint8_t address() const { return address_; }

    bool WriteRegister(uint8_t reg, uint8_t value) const override {
        // Start a write transaction to the current device address.
        wire_->beginTransmission(address_);
        // Send the target register address.
        wire_->write(reg);
        // Send the payload byte for that register.
        wire_->write(value);
        // End the transaction; non-zero means the bus/device did not ACK.
        return wire_->endTransmission() == 0;
    }

    bool ReadRegister(uint8_t reg, uint8_t* value) const override {
        // Single-byte reads are just a 1-length block read.
        return ReadRegisters(reg, value, 1, false);
    }

    // Raw reads are for stream-style I2C devices that are not register indexed.
    bool ReadBytes(uint8_t* data, size_t length) const {
        if (data == nullptr || length == 0 || length > 255) {
            return false;
        }

        const size_t readCount = wire_->requestFrom(address_, static_cast<uint8_t>(length));
        if (readCount != length) {
            return false;
        }

        for (size_t i = 0; i < length; ++i) {
            if (!wire_->available()) {
                return false;
            }
            const int value = wire_->read();
            if (value < 0 || value > 255) {
                return false;
            }
            data[i] = static_cast<uint8_t>(value);
        }
        return true;
    }

    // Some I2C sensors auto-increment when MSB is set on the sub-address.
    bool ReadRegisters(uint8_t reg, uint8_t* data, size_t length, bool autoIncrementMsb) const override {
        if (data == nullptr || length == 0 || length > 255) {
            return false;
        }

        // Compose the starting register address.
        uint8_t startReg = reg;
        // Optionally set the MSB to enable auto-increment for multi-byte reads.
        if (autoIncrementMsb && length > 1) {
            startReg |= 0x80;
        }

        // Write the start register, then keep control of the bus (no stop).
        wire_->beginTransmission(address_);
        wire_->write(startReg);
        if (wire_->endTransmission(false) != 0) {
            return false;
        }

        // Request the desired number of bytes from the device.
        size_t readCount = wire_->requestFrom(address_, static_cast<uint8_t>(length));
        if (readCount != length) {
            return false;
        }

        // Drain the I2C receive buffer into the caller's array.
        for (size_t i = 0; i < length; ++i) {
            if (!wire_->available()) {
                return false;
            }
            const int value = wire_->read();
            if (value < 0 || value > 255) {
                return false;
            }
            data[i] = static_cast<uint8_t>(value);
        }
        return true;
    }

private:
    uint8_t address_;
    TwoWire* wire_;
};

#endif  // HARDWARE_BUS_I2C_DEVICE_H
