#include "Hardware/drivers/Radio.h"

#include <Arduino.h>

#include "Common.h"

namespace {
// TODO(schematic): confirm which HardwareSerial port the XBee is wired to.
// XBee must be pre-configured to 115200 baud before use (factory default: 9600).
HardwareSerial& kXbeeSerial = Serial1;
}  // namespace

bool Radio::Initialize() {
    diagnostics_ = {};
    initialized_ = false;
    serial_ = &kXbeeSerial;
    serial_->begin(Common::XBEE_BAUDRATE);
    initialized_ = true;
    return true;
}

bool Radio::Send(const uint8_t* data, size_t length) {
    if (!initialized_ || serial_ == nullptr) { return false; }
    if (static_cast<size_t>(serial_->availableForWrite()) < length) {
        ++diagnostics_.txErrorCount;
        return false;
    }
    serial_->write(data, length);
    diagnostics_.bytesSent += static_cast<uint32_t>(length);
    return true;
}

int Radio::Receive(uint8_t* buffer, size_t maxLength) {
    if (!initialized_ || serial_ == nullptr) { return 0; }
    int count = 0;
    while (serial_->available() && static_cast<size_t>(count) < maxLength) {
        buffer[count++] = static_cast<uint8_t>(serial_->read());
    }
    diagnostics_.bytesReceived += static_cast<uint32_t>(count);
    return count;
}

Radio::Diagnostics Radio::GetDiagnostics() const {
    return diagnostics_;
}

void Radio::ClearDiagnostics() {
    diagnostics_ = {};
}
