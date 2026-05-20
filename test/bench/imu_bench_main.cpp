// #include <array>

// #include <Arduino.h>

// #include "Common.h"
// #include "Hardware/drivers/Imu.h"

// namespace {
// IMU gImu;
// constexpr unsigned long kPrintPeriodMs = 250;
// constexpr uint32_t kSamplePeriodMicros =
//     static_cast<uint32_t>(IMU::kSamplePeriodSeconds * 1000000.0f);

// void PrintVector3(const Common::Vector3& v) {
//     Serial.print(v.x, 6);
//     Serial.print(',');
//     Serial.print(v.y, 6);
//     Serial.print(',');
//     Serial.print(v.z, 6);
// }

// void PrintFifoDiagnostics(const IMU::FifoDiagnostics& diagnostics) {
//     Serial.print(" diag={overrun=");
//     Serial.print(diagnostics.overrunCount);
//     Serial.print(",nearFull=");
//     Serial.print(diagnostics.nearFullCount);
//     Serial.print(",unexpectedTag=");
//     Serial.print(diagnostics.unexpectedTagCount);
//     Serial.print(",truncated=");
//     Serial.print(diagnostics.truncatedPairCount);
//     Serial.print('}');
// }
// }  // namespace

// void setup() {
//     Serial.begin(115200);
//     delay(1000);
//     Serial.println("imu_bench: starting");

//     const bool ok = gImu.Initialize();
//     Serial.print("imu_bench: init=");
//     Serial.println(ok ? "OK" : "FAIL");
// }

// void loop() {
//     static unsigned long lastPrintMs = 0;
//     const unsigned long nowMs = millis();
//     if ((nowMs - lastPrintMs) < kPrintPeriodMs) {
//         return;
//     }
//     lastPrintMs = nowMs;

//     const Common::IMUData direct = gImu.ReadData();
//     std::array<Common::IMUData, IMU::kFifoWatermarkPairs> burst{};
//     const size_t count = gImu.ReadFifoSamples(burst.data(), burst.size());
//     const IMU::FifoDiagnostics diagnostics = gImu.GetFifoDiagnostics();

//     Serial.print("imu_bench: direct gyro=");
//     PrintVector3(direct.gyroVel);
//     Serial.print(" accel=");
//     PrintVector3(direct.acceleration);
//     Serial.print(" temp_C=");
//     Serial.print(direct.temperatureCelsius, 2);
//     Serial.print(" fifo_count=");
//     Serial.print(count);

//     if (count > 0) {
//         const uint32_t drainTimestampMicros = micros();
//         const uint32_t oldestTimestampMicros =
//             drainTimestampMicros - static_cast<uint32_t>((count - 1U) * kSamplePeriodMicros);
//         Serial.print(" fifo=[");
//         for (size_t i = 0; i < count; ++i) {
//             burst[i].timestampMicros =
//                 oldestTimestampMicros + static_cast<uint32_t>(i * kSamplePeriodMicros);
//             if (i > 0) {
//                 Serial.print(" | ");
//             }
//             Serial.print("t=");
//             Serial.print(burst[i].timestampMicros);
//             Serial.print(" gyro=");
//             PrintVector3(burst[i].gyroVel);
//             Serial.print(" accel=");
//             PrintVector3(burst[i].acceleration);
//         }
//         Serial.print(']');
//     }

//     PrintFifoDiagnostics(diagnostics);
//     Serial.println();
// }
