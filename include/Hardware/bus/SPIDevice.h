#ifndef HARDWARE_BUS_SPI_DEVICE_H
#define HARDWARE_BUS_SPI_DEVICE_H

#include <cstddef>
#include <cstdint>

#include <Arduino.h>
#include <SPI.h>

#include "RegisterBus.h"

// Register-style SPI wrapper with configurable masks for sensor-specific protocols.
class SPIDevice : public IRegisterBus {
public:
    // Constructor captures chip-select pin and SPI protocol details.
    SPIDevice(uint8_t csPin,
              SPIClass& spi = SPI,
              SPISettings settings = SPISettings(1000000, MSBFIRST, SPI_MODE0),
              uint8_t readMask = 0x80,
              uint8_t writeMask = 0x00,
              uint8_t autoIncrementMask = 0x00)
        : csPin_(csPin),
          spi_(&spi),
          settings_(settings),
          readMask_(readMask),
          writeMask_(writeMask),
          autoIncrementMask_(autoIncrementMask) {}

    // Explicit begin keeps control of CS pin ownership in higher-level init.
    void Begin() const {
        // Configure CS high (inactive) before any SPI transactions.
        pinMode(csPin_, OUTPUT);
        digitalWrite(csPin_, HIGH);
        // Initialize the SPI peripheral.
        spi_->begin();
    }

    // Update SPI timing/phase settings per device datasheet.
    void setSettings(const SPISettings& settings) { settings_ = settings; }
    // Some devices set a read bit in the address; mask handles that.
    void setReadMask(uint8_t mask) { readMask_ = mask; }
    // Some devices set a write bit in the address; mask handles that.
    void setWriteMask(uint8_t mask) { writeMask_ = mask; }
    // Some devices need a bit set to auto-increment registers.
    void setAutoIncrementMask(uint8_t mask) { autoIncrementMask_ = mask; }

    // Masking lets us support sensors that encode R/W in the address.
    bool WriteRegister(uint8_t reg, uint8_t value) const override {
        // Take ownership of the SPI bus with the chosen settings.
        spi_->beginTransaction(settings_);
        // Assert CS to select this device.
        digitalWrite(csPin_, LOW);
        // Send the register address with the write mask applied.
        spi_->transfer(reg | writeMask_);
        // Send the payload byte.
        spi_->transfer(value);
        // Deassert CS to end the transfer.
        digitalWrite(csPin_, HIGH);
        // Release the SPI bus for other devices.
        spi_->endTransaction();
        return true;
    }

    bool ReadRegister(uint8_t reg, uint8_t* value) const override {
        // Single-byte reads are just a 1-length block read.
        return ReadRegisters(reg, value, 1, false);
    }

    bool ReadRegisters(uint8_t reg, uint8_t* data, size_t length, bool autoIncrementMsb) const override {
        // Apply the read mask so the device knows this is a read.
        uint8_t startReg = static_cast<uint8_t>(reg | readMask_);
        // Some devices require an extra bit to auto-increment registers.
        if (autoIncrementMsb && length > 1) {
            startReg |= autoIncrementMask_;
        }

        // Begin the SPI transaction and select the device.
        spi_->beginTransaction(settings_);
        digitalWrite(csPin_, LOW);
        // Send the start register, then clock out the data bytes.
        spi_->transfer(startReg);
        for (size_t i = 0; i < length; ++i) {
            data[i] = spi_->transfer(0x00);
        }
        // Deassert CS and end the transaction.
        digitalWrite(csPin_, HIGH);
        spi_->endTransaction();
        return true;
    }

private:
    uint8_t csPin_;
    SPIClass* spi_;
    SPISettings settings_;
    uint8_t readMask_;
    uint8_t writeMask_;
    uint8_t autoIncrementMask_;
};

#endif  // HARDWARE_BUS_SPI_DEVICE_H
