#include <Arduino.h>

#include "Hardware/drivers/Sg90Servo.h"

// SG90 angle-only hardware bench test.
//
// Safety before powering:
// - Use an external servo supply, not the Teensy 5 V pin.
// - Tie servo supply ground to Teensy ground.
// - Start with linkage disconnected or mechanically unloaded.
// - Verify each endpoint before installing the servo into the mechanism.

static constexpr uint8_t kServoPin = 23;
static constexpr unsigned long kSettleMs = 800;

namespace {

Sg90Servo gServo{kServoPin};

void CommandAngle(float degrees) {
    Serial.print(F("Command SG90 angle "));
    Serial.print(degrees, 1);
    Serial.println(F(" deg; attached Servo backend should hold this angle"));

    gServo.SetAngle(degrees);
    delay(kSettleMs);
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n========================================"));
    Serial.println(F("  SG90 Servo Angle Bench Test"));
    Serial.println(F("========================================"));
    Serial.print(F("Signal pin: "));
    Serial.println(kServoPin);
    Serial.println(F("Pulse basis: local SG90 datasheet, conservative 1000/1500/2000 us"));

    const bool ok = gServo.Initialize();
    Serial.print(F("Servo init: "));
    Serial.println(ok ? F("OK") : F("FAIL"));

    if (!ok) {
        while (true) {
            delay(1000);
        }
    }

    const float angles[] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, -30.0f, 210.0f, 90.0f};
    const int count = static_cast<int>(sizeof(angles) / sizeof(angles[0]));
    for (int i = 0; i < count; ++i) {
        CommandAngle(angles[i]);
    }

    Serial.println(F("SG90 bench sequence complete."));
}

void loop() {
    delay(1000);
}
