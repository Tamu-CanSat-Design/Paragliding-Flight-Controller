#ifndef HARDWARE_EXAMPLE_IMU_INTERFACE_H
#define HARDWARE_EXAMPLE_IMU_INTERFACE_H

#include "IMU.h"

// Example interface wrapper for an IMU. This file is a teaching aid only.
// Real interfaces should live in this directory and be used by drivers.
class IExampleImu : public IIMU {
public:
    virtual ~IExampleImu() = default;
};

#endif  // HARDWARE_EXAMPLE_IMU_INTERFACE_H
