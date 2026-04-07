#include <Arduino.h>
#include <Wire.h>
#include <unity.h>

#include "Common.h"
#include "Hardware/bus/I2CDevice.h"
#include "Hardware/drivers/Barometer.h"

namespace {
constexpr uint8_t kBmp390AddressLow = 0x76;
constexpr uint8_t kBmp390AddressHigh = 0x77;
constexpr uint8_t kBmp390ChipIdReg = 0x00;
constexpr uint8_t kBmp390ChipId = 0x60;
constexpr uint8_t kBmp390PwrCtrlReg = 0x1B;
constexpr uint8_t kBmp390OsrReg = 0x1C;
constexpr uint8_t kBmp390OdrReg = 0x1D;
constexpr uint8_t kBmp390ConfigReg = 0x1F;
constexpr uint32_t kReadPeriodMs = 500;
constexpr const char* kBuildDate = __DATE__;
constexpr const char* kBuildTime = __TIME__;

Barometer gBarometer{Wire};
I2CDevice gProbeDevice{0, Wire};

bool ProbeBmp390(uint8_t* detectedAddress, uint8_t* chipId) {
    if (detectedAddress == nullptr || chipId == nullptr) {
        return false;
    }

    const uint8_t candidateAddresses[] = {kBmp390AddressLow, kBmp390AddressHigh};
    for (uint8_t address : candidateAddresses) {
        gProbeDevice.setAddress(address);
        if (gProbeDevice.ReadRegister(kBmp390ChipIdReg, chipId) && *chipId == kBmp390ChipId) {
            *detectedAddress = address;
            return true;
        }
    }

    return false;
}

void PrintRegisterReadback(uint8_t address) {
    gProbeDevice.setAddress(address);

    uint8_t pwrCtrl = 0;
    uint8_t osr = 0;
    uint8_t odr = 0;
    uint8_t config = 0;

    const bool okPwrCtrl = gProbeDevice.ReadRegister(kBmp390PwrCtrlReg, &pwrCtrl);
    const bool okOsr = gProbeDevice.ReadRegister(kBmp390OsrReg, &osr);
    const bool okOdr = gProbeDevice.ReadRegister(kBmp390OdrReg, &odr);
    const bool okConfig = gProbeDevice.ReadRegister(kBmp390ConfigReg, &config);

    Serial.println("Register readback after driver init:");
    Serial.printf("  PWR_CTRL (0x1B): %s 0x%02X\n", okPwrCtrl ? "" : "READ_FAIL", pwrCtrl);
    Serial.printf("  OSR      (0x1C): %s 0x%02X\n", okOsr ? "" : "READ_FAIL", osr);
    Serial.printf("  ODR      (0x1D): %s 0x%02X\n", okOdr ? "" : "READ_FAIL", odr);
    Serial.printf("  CONFIG   (0x1F): %s 0x%02X\n", okConfig ? "" : "READ_FAIL", config);
    Serial.println("Expected values for this driver:");
    Serial.println("  PWR_CTRL=0x33  OSR=0x0B  ODR=0x03  CONFIG=0x04");
}

void PrintSanityCheck(const Common::BarometerData& data) {
    bool sane = true;
    if (data.pressure < 300.0f || data.pressure > 1200.0f) {
        sane = false;
    }
    if (data.temperature < -40.0f || data.temperature > 85.0f) {
        sane = false;
    }

    Serial.printf("Sanity: %s\n", sane ? "PASS" : "CHECK_VALUES");
}

void test_bmp390_hardware_harness_builds() {
    TEST_ASSERT_TRUE(true);
}
}  // namespace

extern "C" void setup() {
    Serial.begin(115200);
    delay(1000);

    UNITY_BEGIN();
    RUN_TEST(test_bmp390_hardware_harness_builds);
    UNITY_END();

    Wire.begin();
    Wire.setClock(Common::I2C_BAUDRATE);

    Serial.println();
    Serial.println("BMP390 sensor testbench");
    Serial.printf("Build: %s %s\n", kBuildDate, kBuildTime);
    Serial.println("Teensy 4.1 default I2C pins: SDA=18, SCL=19");
    Serial.println("Hold the sensor still while the relative-altitude baseline is captured.");

    uint8_t address = 0;
    uint8_t chipId = 0;
    if (ProbeBmp390(&address, &chipId)) {
        Serial.printf("Probe: PASS  address=0x%02X  chip_id=0x%02X\n", address, chipId);
    } else {
        Serial.println("Probe: FAIL  no BMP390 found at 0x76 or 0x77");
    }

    Serial.println("Driver init started: waiting for a stable pressure baseline in the background...");
    const bool initOk = gBarometer.Initialize();
    Serial.printf("Driver init: %s\n", initOk ? "PASS" : "FAIL");

    if (initOk && address != 0) {
        PrintRegisterReadback(address);
    }
}

extern "C" void loop() {
    const Common::BarometerData data = gBarometer.ReadData();
    if (gBarometer.LastReadSucceeded()) {
        (void)gBarometer.ProcessReferencePressureSample(data, millis());
    } else {
        (void)gBarometer.ProcessReferencePressureSample(Common::BarometerData{}, millis());
    }

    Serial.printf("Pressure: %.2f hPa  Temperature: %.2f C  Altitude: %.2f m\n",
                  data.pressure,
                  data.temperature,
                  data.altitude);
    PrintSanityCheck(data);
    delay(kReadPeriodMs);
}
