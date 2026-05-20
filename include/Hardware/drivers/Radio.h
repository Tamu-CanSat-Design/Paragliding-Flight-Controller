// #ifndef HARDWARE_RADIO_DRIVER_H
// #define HARDWARE_RADIO_DRIVER_H

// #include "../interfaces/RadioInterface.h"

// // Forward declaration avoids pulling <Arduino.h> into every translation unit
// // that includes this header.
// class HardwareSerial;

// class Radio : public IRadio {
// public:
//     struct Diagnostics {
//         uint32_t bytesSent = 0;
//         uint32_t bytesReceived = 0;
//         uint32_t txErrorCount = 0;
//     };

//     bool Initialize() override;
//     bool Send(const uint8_t* data, size_t length) override;
//     int Receive(uint8_t* buffer, size_t maxLength) override;
//     Diagnostics GetDiagnostics() const;
//     void ClearDiagnostics();

// private:
//     Diagnostics diagnostics_{};
//     // Assigned to the XBee serial port in Initialize().
//     // TODO(schematic): confirm which HardwareSerial instance (Serial1..8).
//     HardwareSerial* serial_ = nullptr;
//     bool initialized_ = false;
// };

// #endif  // HARDWARE_RADIO_DRIVER_H
