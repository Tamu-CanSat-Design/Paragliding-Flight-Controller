#include "interfaces/VoltageSensorInterface.h"

class VoltageSensor : public IVoltageSensor {
public:
    bool Initialize();
    Common::VoltageData ReadData();
};

bool VoltageSensor::Initialize() {
    return true;
}

Common::VoltageData VoltageSensor::ReadData() {
    return {};
}
