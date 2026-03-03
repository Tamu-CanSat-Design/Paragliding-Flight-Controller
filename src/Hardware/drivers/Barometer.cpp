#include "Hardware/drivers/Barometer.h"

#include <Arduino.h>
#include <Wire.h>

#include "bmp3.h"
#include "Common.h"

static bfs::Bmp3 g_bmp;

bool Barometer::Initialize() {
  initialized_ = false;
  data_ = {};
  calib_ = {};  // kept to match the header

  // I2C setup
  Wire.begin();
  Wire.setClock(Common::I2C_BAUDRATE);

  // Configure bmp3 to use this I2C bus and address (0x76 default).
  bfs::Bmp3::I2cAddr addr{0x76};
  g_bmp.Config(&Wire, addr);

  if (!g_bmp.Begin()) {        // real Begin() from bmp3.cpp
    Serial.println("Error initializing communication with BMP390");
    return false;
  }

  initialized_ = true;
  return true;
}

Common::BarometerData Barometer::ReadData() {
  Common::BarometerData d{};

  d.temperature = g_bmp.die_temp_c();
  float pressure_data = g_bmp.pres_pa();
  d.pressure    = pressure_data;
  d.altitude    = PressureToAltitude(pressure_data);

  return d;
}

float Barometer::PressureToAltitude(float pressure_hpa) {
  return 44330.0f * (1.0f - powf(pressure_hpa / 1013.25f, 0.1903f));
}
