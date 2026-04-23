#include <Arduino.h>

#include "Common.h"
#include "Hardware/drivers/Radio.h"

namespace {
Radio gRadio;
constexpr unsigned long kTxPeriodMs = 1000;
constexpr unsigned long kRxPollMs   = 100;
const char kTestPacket[] = "radio_bench: test\n";
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("radio_bench: starting");
    Serial.print("radio_bench: xbee_baud=");
    Serial.println(Common::XBEE_BAUDRATE);

    const bool ok = gRadio.Initialize();
    Serial.print("radio_bench: init=");
    Serial.println(ok ? "OK" : "FAIL");
}

void loop() {
    static unsigned long lastTxMs  = 0;
    static unsigned long lastRxMs  = 0;

    const unsigned long nowMs = millis();

    // Drain received bytes at high frequency.
    if ((nowMs - lastRxMs) >= kRxPollMs) {
        lastRxMs = nowMs;
        uint8_t rxBuf[64] = {};
        const int n = gRadio.Receive(rxBuf, sizeof(rxBuf) - 1U);
        if (n > 0) {
            Serial.print("radio_bench: rx=");
            Serial.write(rxBuf, static_cast<size_t>(n));
            Serial.println();
        }
    }

    // Send test packet once per second.
    if ((nowMs - lastTxMs) >= kTxPeriodMs) {
        lastTxMs = nowMs;

        const auto* packet = reinterpret_cast<const uint8_t*>(kTestPacket);
        const bool sent = gRadio.Send(packet, sizeof(kTestPacket) - 1U);
        const Radio::Diagnostics diag = gRadio.GetDiagnostics();

        Serial.print("radio_bench: tx=");
        Serial.print(sent ? "OK" : "FAIL");
        Serial.print(" diag={sent=");
        Serial.print(diag.bytesSent);
        Serial.print(",recv=");
        Serial.print(diag.bytesReceived);
        Serial.print(",txErr=");
        Serial.print(diag.txErrorCount);
        Serial.println("}");
    }
}
