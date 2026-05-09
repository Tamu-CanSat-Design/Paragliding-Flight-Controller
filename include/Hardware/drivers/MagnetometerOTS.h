#ifndef HARDWARE_MAGNETOMETEROTS_DRIVER_H
#define HARDWARE_MAGNETOMETEROTS_DRIVER_H

#include "../bus/I2CDevice.h"
#include "../interfaces/MagnetometerInterface.h"
#include "Common.h"
#include <IIS2MDCSensor.h>
class MagnetometerOTS : public IMagnetometer{
public:


    static constexpr const char* kSensorName = "IIS2MDC";
    static constexpr float kSensitivity = 1.5f / 1000.0f; // example sensitivity in mG/LSB, adjust based on actual sensor settings
    // static constexpr uint32_t kOdrHz = 100;
    // static constexpr uint32_t kOdrPeriodMs = 1000u / kOdrHz;

    const char* SensorName() const override { return kSensorName; }
    bool Initialize() override;
    Common::MagData ReadData() override;


private: 
    Common::MagData data_{};
    IIS2MDCSensor device_{&Wire1};
    bool initialized_ = false;
};




#endif // HARDWARE_MAGNETOMETEROTS_DRIVER_H
