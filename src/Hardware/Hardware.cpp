#include "Hardware/Hardware.h"

namespace {
// Default PWM pins for the three mechanical release servos.
// These can be overridden at build time with RELEASE_SERVO_0_PIN / _1_PIN / _2_PIN.
#ifndef RELEASE_SERVO_0_PIN
constexpr uint8_t kReleaseServo0Pin = 2;
#else
constexpr uint8_t kReleaseServo0Pin = static_cast<uint8_t>(RELEASE_SERVO_0_PIN);
#endif

#ifndef RELEASE_SERVO_1_PIN
constexpr uint8_t kReleaseServo1Pin = 3;
#else
constexpr uint8_t kReleaseServo1Pin = static_cast<uint8_t>(RELEASE_SERVO_1_PIN);
#endif

#ifndef RELEASE_SERVO_2_PIN
constexpr uint8_t kReleaseServo2Pin = 4;
#else
constexpr uint8_t kReleaseServo2Pin = static_cast<uint8_t>(RELEASE_SERVO_2_PIN);
#endif

constexpr std::array<uint8_t, 3> kReleaseServoPins{
    {kReleaseServo0Pin, kReleaseServo1Pin, kReleaseServo2Pin}
};

// Default PWM pins for the steering servos.
// These can be overridden at build time with STEERING_SERVO_0_PIN / _1_PIN.
#ifndef STEERING_SERVO_0_PIN
constexpr uint8_t kSteeringServo0Pin = 5;
#else
constexpr uint8_t kSteeringServo0Pin = static_cast<uint8_t>(STEERING_SERVO_0_PIN);
#endif

#ifndef STEERING_SERVO_1_PIN
constexpr uint8_t kSteeringServo1Pin = 6;
#else
constexpr uint8_t kSteeringServo1Pin = static_cast<uint8_t>(STEERING_SERVO_1_PIN);
#endif

constexpr std::array<uint8_t, 2> kSteeringServoPins{
    {kSteeringServo0Pin, kSteeringServo1Pin}
};

// Default control pin for the audio beacon driver stage.
// Override with AUDIO_BEACON_PIN for board-specific routing.
#ifndef AUDIO_BEACON_PIN
constexpr uint8_t kAudioBeaconPin = 7;
#else
constexpr uint8_t kAudioBeaconPin = static_cast<uint8_t>(AUDIO_BEACON_PIN);
#endif

// Default polarity for the beacon control output.
// Set AUDIO_BEACON_ACTIVE_HIGH=0 if hardware driver is active-low.
#ifndef AUDIO_BEACON_ACTIVE_HIGH
constexpr bool kAudioBeaconActiveHigh = true;
#else
constexpr bool kAudioBeaconActiveHigh = (AUDIO_BEACON_ACTIVE_HIGH != 0);
#endif
}  // namespace

namespace Hardware {

    // Global hardware manager instance. Tasks can access this to read sensors, send radio messages, etc.
    // This is defined here in the .cpp file to avoid circular dependencies with the header.
    
    Barometer barometer_;
    IMU imu_;
    TeseoGPS gps_;
    Lidar lidar_;
    VoltageSensor voltageSensor_;
    Radio radio_;
    AudioBeacon audioBeacon_;
    std::array<Camera, 2> cameras_;
    std::array<ReleaseServo, 3> mechServos_;
    std::array<SteeringServo, 2> steeringServos_;
    I2CDevice i2c1_{};
    I2CDevice i2c2_{};

    static_assert(std::tuple_size<decltype(mechServos_)>::value == kReleaseServoPins.size(),
                  "Release servo pin count must match mechServos_ array length.");
    static_assert(std::tuple_size<decltype(steeringServos_)>::value == kSteeringServoPins.size(),
                  "Steering servo pin count must match steeringServos_ array length.");




    bool InitializeAllSensors() {
        // Configure pin-dependent devices before any Initialize() calls.
        // This keeps all drivers in a deterministic startup state.
        audioBeacon_.ConfigurePin(kAudioBeaconPin);
        audioBeacon_.ConfigureActiveLevel(kAudioBeaconActiveHigh);

        for (size_t i = 0; i < mechServos_.size(); ++i) {
            mechServos_[i].ConfigurePin(kReleaseServoPins[i]);
        }
        for (size_t i = 0; i < steeringServos_.size(); ++i) {
            steeringServos_[i].ConfigurePin(kSteeringServoPins[i]);
        }

        // Initialize all sensing devices first so mission logic can trust input data.
        bool ok = true;
        ok = barometer_.Initialize() && ok;
        ok = imu_.Initialize() && ok;
        ok = gps_.Initialize() && ok;
        ok = lidar_.Initialize() && ok;
        ok = voltageSensor_.Initialize() && ok;
        for (size_t i = 0; i < cameras_.size(); ++i) {
            ok = cameras_[i].Initialize() && ok;
        }

        // Initialize communications and actuators after sensors.
        ok = radio_.Initialize() && ok;
        ok = audioBeacon_.Initialize() && ok;

        for (size_t i = 0; i < mechServos_.size(); ++i) {
            ok = mechServos_[i].Initialize() && ok;
        }

        for (size_t i = 0; i < steeringServos_.size(); ++i) {
            ok = steeringServos_[i].Initialize() && ok;
        }
        return ok;
    }
}
