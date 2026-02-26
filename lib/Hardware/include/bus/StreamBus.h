#ifndef HARDWARE_BUS_STREAM_BUS_H
#define HARDWARE_BUS_STREAM_BUS_H

#include <cstddef>
#include <cstdint>

// Stream-style interface for UART-like devices with byte-oriented I/O.
class IStreamBus {
public:
    virtual ~IStreamBus() = default;
    // Mirrors a subset of Arduino Stream for portability across platforms.
    // Keep methods small so different serial implementations can plug in.
    // Write raw bytes to the stream (returns bytes written).
    virtual size_t Write(const uint8_t* data, size_t length) = 0;
    // Report how many bytes are ready to be read.
    virtual int Available() = 0;
    // Read a single byte (or -1 if none are available).
    virtual int Read() = 0;
    // Read up to length bytes into buffer (returns bytes read).
    virtual size_t ReadBytes(uint8_t* buffer, size_t length) = 0;
};

#endif  // HARDWARE_BUS_STREAM_BUS_H
