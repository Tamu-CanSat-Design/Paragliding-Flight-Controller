#include "Hardware/drivers/MagnetometerOTS.h"

bool MagnetometerOTS::Initialize() {
    
    if (device_.begin() != IIS2MDC_OK) {
        initialized_ = false;
        return false;
    }
    



    if(device_.Enable() != IIS2MDC_OK) {
        initialized_ = false;
        return false;
    }

    if(device_.SetOutputDataRate(IIS2MDC_ODR_100Hz) != IIS2MDC_OK) {
        initialized_ = false;
        return false;
    }

    initialized_ = true;
    return true;
}

Common::MagData MagnetometerOTS::ReadData() {
    if (!initialized_) {
        return data_;
    }

    int16_t raw_data[3];
    if (device_.GetAxesRaw(raw_data) != IIS2MDC_OK) {
        return data_;
    }
    

    data_.magX = static_cast<float>(raw_data[0]) * kSensitivity;
    data_.magY = static_cast<float>(raw_data[1]) * kSensitivity;
    data_.magZ = static_cast<float>(raw_data[2]) * kSensitivity;

    return data_;
}