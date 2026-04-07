#ifndef HARDWARE_BAROMETER_DRIVER_H
#define HARDWARE_BAROMETER_DRIVER_H

#include <array>
#include <cstddef>
#include <cstdint>

#include "../bus/I2CDevice.h"
#include "../interfaces/BarometerInterface.h"

class Barometer : public IBarometer {
public:
    enum class ReferenceCaptureStatus : uint8_t {
        kIdle = 0,
        kPending = 1,
        kReady = 2,
        kTimedOut = 3,
    };

    static constexpr const char* kSensorName = "BMP390";

    explicit Barometer(TwoWire& wire);
    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::BarometerData ReadData() override;
    bool RezeroReferencePressure();
    float GetReferencePressureHpa() const { return referencePressureHpa_; }
    void StartReferencePressureCapture(uint32_t startMs);
    ReferenceCaptureStatus ProcessReferencePressureSample(const Common::BarometerData& sample, uint32_t nowMs);
    bool ReferencePressureCaptureInProgress() const { return referenceCaptureInProgress_; }
    bool LastReadSucceeded() const { return lastReadSucceeded_; }

private:
    static constexpr size_t kReferenceWindowSampleCount = 50;

    struct Calibration {
        float par_t1 = 0.0f;
        float par_t2 = 0.0f;
        float par_t3 = 0.0f;
        float par_p1 = 0.0f;
        float par_p2 = 0.0f;
        float par_p3 = 0.0f;
        float par_p4 = 0.0f;
        float par_p5 = 0.0f;
        float par_p6 = 0.0f;
        float par_p7 = 0.0f;
        float par_p8 = 0.0f;
        float par_p9 = 0.0f;
        float par_p10 = 0.0f;
        float par_p11 = 0.0f;
        float t_lin = 0.0f;
    };

    bool SoftReset();
    bool ReadCalibration();
    bool Configure();
    bool ReadRaw(uint32_t* raw_press, uint32_t* raw_temp);
    float CompensateTemperature(uint32_t raw_temp);
    float CompensatePressure(uint32_t raw_press);
    static float PressureToAltitude(float pressure_hpa, float reference_pressure_hpa);

    Common::BarometerData data_{};
    Calibration calib_{};
    I2CDevice device_;
    float referencePressureHpa_ = 0.0f;
    bool initialized_ = false;
    bool lastReadSucceeded_ = false;
    bool referenceCaptureInProgress_ = false;
    bool referenceCaptureHasSample_ = false;
    uint32_t referenceCaptureStartMs_ = 0;
    uint32_t lastReferenceSampleMs_ = 0;
    size_t referenceSampleCount_ = 0;
    size_t nextReferenceSampleIndex_ = 0;
    std::array<float, kReferenceWindowSampleCount> referenceWindowPressures_{};
};

#endif  // HARDWARE_BAROMETER_DRIVER_H
