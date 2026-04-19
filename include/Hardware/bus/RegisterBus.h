#ifndef HARDWARE_BUS_REGISTER_BUS_H
#define HARDWARE_BUS_REGISTER_BUS_H

#include <cstddef>
#include <cstdint>

// Common register-based bus interface so drivers can swap I2C/SPI without rewrites.
class IRegisterBus {
public:
    virtual ~IRegisterBus() = default;
    // Minimal register ops most sensors need; keeps driver logic bus-agnostic.
    // Each call returns false on bus errors so drivers can retry or fail fast.
    // Write a single register byte (common for config or mode bits).
    virtual bool WriteRegister(uint8_t reg, uint8_t value) const = 0;
    // Read a single register byte; used for IDs, status flags, etc.
    virtual bool ReadRegister(uint8_t reg, uint8_t* value) const = 0;
    // Read a block of registers; autoIncrementMsb is protocol-specific.
    virtual bool ReadRegisters(uint8_t reg, uint8_t* data, size_t length, bool autoIncrementMsb) const = 0;
};

#endif  // HARDWARE_BUS_REGISTER_BUS_H
