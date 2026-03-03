#include "Common.h"
#include "Hardware/drivers/Lidar.h"
#include "LIDARLite.h"
#include <Wire.h>

static LIDARLite g_lidar;

bool Lidar::Initialize() {
  Wire.begin();
  Wire.setClock(Common::I2C_BAUDRATE);

  g_lidar.begin(0, true);

  return true;
}

Common::LidarData Lidar::ReadData() {
  Common::LidarData data{};

  uint16_t dist_cm = g_lidar.distance();
  data.distanceMeters = static_cast<float>(dist_cm) / 100.0f;

  return data;
}
