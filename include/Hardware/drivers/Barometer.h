#ifndef HARDWARE_BAROMETER_DRIVER_H
#define HARDWARE_BAROMETER_DRIVER_H

#include <cstdint>

#include "../bus/I2CDevice.h"
#include "../interfaces/BarometerInterface.h"

class Barometer : public IBarometer {
public:
    static constexpr const char* kSensorName = "BMP390";

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::BarometerData ReadData() override;

private:
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
    float PressureToAltitude(float pressure_hpa);

    Common::BarometerData data_{};
    Calibration calib_{};
    I2CDevice device_{};
    bool initialized_ = false;
};

#endif  // HARDWARE_BAROMETER_DRIVER_H
