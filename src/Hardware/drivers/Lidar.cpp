#include "Common.h"
#include "Hardware/drivers/Lidar.h"
#include "LIDARLite.h"
#include <Wire.h>

static LIDARLite g_lidar;

bool Lidar::Initialize() {
  Wire.begin();
  Wire.setClock(Common::I2C_BAUDRATE);

  g_lidar.begin(0xC5, true);

  return true;
}

Common::LidarData Lidar::ReadData() {
  Common::LidarData data{};

  data.distanceMeters = g_lidar.distance();

  return data;
}
