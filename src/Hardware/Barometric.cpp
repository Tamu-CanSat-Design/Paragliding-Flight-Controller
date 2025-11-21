#include "Hardware/Barometer.h"
class BarometerBMP390 : public Barometer {

public:
    void Initialize() override {
        // Initialization code for BMP390 barometer
        pressure = 1013.25f; // Example initial pressure in hPa
        temperature = 25.0f; // Example initial temperature in Celsius
    }

    Common::Vector3 ReadData() override {
        // Simulated reading from the BMP390 barometer
        // In a real implementation, this would interface with hardware
        Common::Vector3 data;
        data.x = pressure;     // Pressure in hPa
        data.y = temperature;  // Temperature in Celsius
        data.z = 0.0f;         // Placeholder for altitude or other data
        return data;
    }


private:

    float pressure;
    float temperature;
    
};