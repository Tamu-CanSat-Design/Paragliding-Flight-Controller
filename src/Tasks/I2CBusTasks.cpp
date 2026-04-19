#include "TaskHandler.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace {
// The mission sensor budget from the current requirements screenshot is:
//   BMP390      -> 50 Hz internal
//   ISM330DHCX  -> 104 Hz internal
//   IIS2MDC     -> 50 Hz internal
//   TESEO LIV3R -> 5 Hz internal
//   INA260      -> 7 Hz effective
//   LIDAR-Lite  -> 100 Hz internal
//
// These two bus tasks are the only code that touches the physical I2C buses.
// Each task wakes once per RTOS tick, checks which sensors are due, reads only
// those sensors, then pushes the freshest values into:
//   1. the per-sensor queues consumed by NavigationTask
//   2. the shared gSnapshot consumed by telemetry/logging/mission logic
//
// The RTOS gives us timing in whole ticks, not arbitrary fractions of a
// millisecond. Because of that, rates like 50 Hz, 20 Hz, and 5 Hz map cleanly
// to integer millisecond periods, while the IMU does not. ST's documented
// accel/gyro ODRs are 12.5, 26, 52, 104, 208, 416, 833, 1666, 3332, and
// 6667 Hz, so there is not a "faster but clean integer-ms" option above 104 Hz.
// The IMU now uses the ISM330DHCX FIFO to avoid that limitation. The sensor runs
// internally at 416 Hz and batches accel+gyro words into FIFO. Because the PCB
// does not route the IMU interrupt pin back to the MCU, I2C2BusTask polls FIFO
// status every 10 ms and drains up to 4 accel+gyro pairs per service. That
// keeps the host-side service cadence at roughly 104 Hz without requiring exact
// 416 Hz task wakeups. Each FIFO drain is forwarded to NavigationTask as one
// ordered IMU batch carrying reconstructed per-sample timestamps so the MEKF can
// process the samples sequentially without losing the original sensor cadence.
constexpr float kDefaultReferencePressureHpa = 1013.25f;
constexpr TickType_t kBusTaskLoopPeriod = 1;
constexpr TickType_t kBarometerPeriodTicks = pdMS_TO_TICKS(20);   // 50 Hz
constexpr TickType_t kGpsPeriodTicks = pdMS_TO_TICKS(200);        // 5 Hz
constexpr TickType_t kLidarPeriodTicks = pdMS_TO_TICKS(10);       // 100 Hz
constexpr TickType_t kMagPeriodTicks = pdMS_TO_TICKS(20);         // 50 Hz
constexpr TickType_t kVoltagePeriodTicks = pdMS_TO_TICKS(143);    // nearest tick period to ~7 Hz
constexpr TickType_t kImuPollPeriodTicks = pdMS_TO_TICKS(10);     // poll FIFO every 10 ms for 4-sample bursts
constexpr uint32_t kImuSamplePeriodMicros = static_cast<uint32_t>(IMU::kSamplePeriodSeconds * 1000000.0f + 0.5f);

bool IsTimeDue(TickType_t now, TickType_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

// After a sensor publishes, move its deadline forward by exactly one period. If
// the task ran late, keep stepping the deadline forward until it lands in the
// future again. This drops missed opportunities instead of trying to "catch up"
// with a burst of back-to-back I2C reads.
void AdvanceDeadline(TickType_t* deadline, TickType_t periodTicks, TickType_t now) {
    if (deadline == nullptr || periodTicks == 0) {
        return;
    }

    do {
        *deadline += periodTicks;
    } while (IsTimeDue(now, *deadline));
}

// Reference pressure is calibrated by CAL and shared with the bus task so the
// published barometer altitude is relative to the current launch baseline rather
// than a fixed sea-level constant baked into the sensor driver.
float ReadReferencePressureHpa() {
    float referencePressure = kDefaultReferencePressureHpa;

    if (SharedData::gReferencePressureMutex != nullptr
        && xSemaphoreTake(SharedData::gReferencePressureMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        referencePressure = SharedData::gReferencePressureHpa;
        xSemaphoreGive(SharedData::gReferencePressureMutex);
    }

    return referencePressure;
}

// Convert the current pressure sample into altitude relative to the stored
// reference pressure. This is done at the publication boundary so the raw driver
// stays responsible only for physical sensor units.
float PressureToRelativeAltitude(float pressureHpa, float referencePressureHpa) {
    if (pressureHpa <= 0.0f || referencePressureHpa <= 0.0f) {
        return 0.0f;
    }

    const float ratio = pressureHpa / referencePressureHpa;
    return 44330.0f * (1.0f - std::pow(ratio, 0.190294957f));
}

template <typename T>
void UpdateSnapshot(T Common::SensorSnapshot::* field, const T& value) {
    if (SharedData::gSnapshotMutex == nullptr) {
        return;
    }

    if (xSemaphoreTake(SharedData::gSnapshotMutex, 0) == pdTRUE) {
        SharedData::gSnapshot.*field = value;
        xSemaphoreGive(SharedData::gSnapshotMutex);
    }
}

}  // namespace

void Tasks::I2C1BusTask(void* pvParameters) {
    (void)pvParameters;

    // Each task keeps its own last published value so a failed read does not
    // inject zeros into the rest of the flight stack.
    Common::BarometerData latestBaro = {};
    Common::GPSData latestGps = {};
    Common::LidarData latestLidar = {};

    // lastWakeTime drives vTaskDelayUntil so the task wakes on a stable RTOS
    // cadence instead of drifting over time.
    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t nextBarometerTick = lastWakeTime;
    TickType_t nextGpsTick = lastWakeTime;
    TickType_t nextLidarTick = lastWakeTime;

    while (true) {
        const TickType_t now = xTaskGetTickCount();

        // Barometer ownership lives here. After a read, the task computes the
        // relative altitude, queues the sample for NavigationTask, and mirrors it
        // into gSnapshot for telemetry/logging/mission logic.
        if (IsTimeDue(now, nextBarometerTick)) {
            latestBaro = Hardware::barometer_.ReadData();
            latestBaro.altitude = PressureToRelativeAltitude(latestBaro.pressure, ReadReferencePressureHpa());
            xQueueSend(SharedData::gI2C1BarometerQueue, &latestBaro, portMAX_DELAY); // TODO: Don't make this a blocking send
            UpdateSnapshot(&Common::SensorSnapshot::barometerData, latestBaro);
            AdvanceDeadline(&nextBarometerTick, kBarometerPeriodTicks, now);
        }

        // GPS is intentionally slow compared to the rest of the bus. The queue
        // receives every internal sample; telemetry later decides what subset to
        // downlink at 1 Hz.
        if (IsTimeDue(now, nextGpsTick)) {
            latestGps = Hardware::gps_.ReadData();
            xQueueSend(SharedData::gI2C1GPSQueue, &latestGps, portMAX_DELAY); //TODO: Don't make this a blocking send
            UpdateSnapshot(&Common::SensorSnapshot::gpsData, latestGps);
            AdvanceDeadline(&nextGpsTick, kGpsPeriodTicks, now);
        }

        // Lidar is bus-owned here as well. Just like the other sensors, the read
        // result is published to both the queue path and the snapshot path.
        if (IsTimeDue(now, nextLidarTick)) {
            latestLidar = Hardware::lidar_.ReadData();
            xQueueSend(SharedData::gI2C1LidarQueue, &latestLidar, portMAX_DELAY); //TODO: Don't make this a blocking send
            UpdateSnapshot(&Common::SensorSnapshot::lidarData, latestLidar);
            AdvanceDeadline(&nextLidarTick, kLidarPeriodTicks, now);
        }

        // One-tick pacing keeps bus ownership responsive while still giving the
        // scheduler room to run higher-priority tasks.
        vTaskDelayUntil(&lastWakeTime, kBusTaskLoopPeriod);
    }
}

void Tasks::I2C2BusTask(void* pvParameters) {
    (void)pvParameters;

    // The IMU path drains FIFO bursts from the ISM330DHCX. Each burst can contain
    // multiple accel+gyro samples captured at the sensor's 416 Hz internal ODR.
    // The magnetometer is a separate die, so it still uses its own direct 50 Hz
    // read path.
    std::array<Common::IMUData, IMU::kFifoWatermarkPairs> imuBurst{};
    Common::IMUBatch imuBatch = {};
    Common::IMUData latestImu = {};
    Common::MagData latestMag = {};
    Common::VoltageData latestVoltage = {};

    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t nextImuPollTick = lastWakeTime;
    TickType_t nextMagTick = lastWakeTime;
    TickType_t nextVoltageTick = lastWakeTime;

    while (true) {
        const TickType_t now = xTaskGetTickCount();

        // Drain all currently available FIFO pairs up to the burst buffer size.
        // The batch keeps the FIFO ordering intact and attaches reconstructed
        // sample timestamps spaced by the IMU's nominal 416 Hz sample period.
        // This path is intentionally polled because the current PCB does not
        // route the IMU interrupt trace.
        if (IsTimeDue(now, nextImuPollTick)) {
            const size_t sampleCount = Hardware::imu_.ReadFifoSamples(imuBurst.data(), imuBurst.size());
            if (sampleCount > 0) {
                imuBatch = {};
                imuBatch.count = static_cast<uint8_t>(sampleCount);
                imuBatch.nominalSamplePeriodMicros = kImuSamplePeriodMicros;

                const uint32_t drainTimestampMicros = micros();
                // FIFO does not hand us hardware timestamps here, so reconstruct
                // the batch timeline by anchoring the newest sample at drain time
                // and stepping backward at the nominal 416 Hz cadence.
                const uint32_t oldestSampleTimestampMicros =
                    drainTimestampMicros - static_cast<uint32_t>((sampleCount - 1) * kImuSamplePeriodMicros);

                for (size_t i = 0; i < sampleCount; ++i) {
                    latestImu = imuBurst[i];
                    imuBatch.samples[i].sample = latestImu;
                    imuBatch.samples[i].timestampMicros =
                        oldestSampleTimestampMicros + static_cast<uint32_t>(i * kImuSamplePeriodMicros);
                }

                xQueueSend(SharedData::gI2C2IMUQueue, &imuBatch, portMAX_DELAY); //TODO: Don't make this a blocking send
                if(xSemaphoreTake(SharedData::gRollAngularVelocityMutex, 0) == pdTRUE){
                    SharedData::gRollAngularVelocity = latestImu.gyroVel.x;
                    xSemaphoreGive(SharedData::gRollAngularVelocityMutex);
                }
            
                }
                // Snapshot keeps only the freshest IMU sample; the full ordered
                // history for MEKF propagation lives in the batch queue.
                UpdateSnapshot(&Common::SensorSnapshot::imuData, imuBatch.samples[sampleCount - 1].sample);
            }
            AdvanceDeadline(&nextImuPollTick, kImuPollPeriodTicks, now);
        

        // The magnetometer remains a separate 50 Hz read on the companion die.
        if (IsTimeDue(now, nextMagTick)) {
            latestMag = Hardware::imu_.ReadMagData();
            xQueueSend(SharedData::gI2C2MagQueue, &latestMag, portMAX_DELAY); // TODO: Don't make this a blocking send
            UpdateSnapshot(&Common::SensorSnapshot::magData, latestMag);
            AdvanceDeadline(&nextMagTick, kMagPeriodTicks, now);
        }

        // Power telemetry is much slower than the inertial sensors, so it gets
        // its own low-rate schedule on the second I2C bus.
        if (IsTimeDue(now, nextVoltageTick)) {
            latestVoltage = Hardware::voltageSensor_.ReadData();
            xQueueSend(SharedData::gI2C2VoltageQueue, &latestVoltage, portMAX_DELAY); // TODO: Don't make this a blocking send
            UpdateSnapshot(&Common::SensorSnapshot::voltageData, latestVoltage);
            AdvanceDeadline(&nextVoltageTick, kVoltagePeriodTicks, now);
        }

        vTaskDelayUntil(&lastWakeTime, kBusTaskLoopPeriod);
    }
}

