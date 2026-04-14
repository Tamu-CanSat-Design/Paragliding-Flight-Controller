#ifndef COMMON_H
#define COMMON_H
#include <cstdio>
#include <optional>

#include <Arduino.h>

namespace Common {

    inline constexpr int I2C_BAUDRATE = 400000;
    inline constexpr int XBEE_BAUDRATE = 115200;
    inline constexpr uint16_t TEAM_ID = 1030;
    inline constexpr unsigned long TELEMETRY_INTERVAL_MS = 1000;
    inline constexpr size_t COMMAND_BUFFER_LENGTH = 32;

    inline constexpr uint8_t VOLTAGE_PIN = 48;
    inline constexpr uint8_t CURRENT_PIN = 49;
    inline constexpr uint8_t BAROMETER_PIN = 44;
    inline constexpr uint8_t IMU_PIN = 42;
    inline constexpr uint8_t GPS_PIN = 50;
    inline constexpr uint8_t MAGNETOMETER_PIN = 43;
    inline constexpr uint8_t LIDAR_PIN = 45;
    inline constexpr uint8_t SD_PIN = 46;

    




    inline constexpr const char LOG_FILE[12] = "datalog.csv";

    enum class FlightState {
        BOOTUP = 0,
        STANDBY = 1,
        ASCENT = 2,
        DESCENT = 3,
        PAYLOAD_RELEASE = 4,
        RECOVERY = 5
    };
    inline static const char* const FlightStateNames[] = {
        "BOOTUP",
        "STANDBY",
        "ASCENT",
        "DESCENT",
        "PAYLOAD_RELEASE",
        "RECOVERY"
    };

    
    inline const char* GetFlightStateName(FlightState state) {
        if (state >= FlightState::BOOTUP && state <= FlightState::RECOVERY) {
            return FlightStateNames[static_cast<int>(state)];
        }
        return "UNKNOWN";
    }
    enum CommandType {
        CX=0,
        ST=1,
        SIM=2,
        SIMP=3,
        CAL=4,
        EGREL=5,
        LDAR=6,
        AUD=7,
        UNKNOWN=8
    };

    inline static const char* CommandTable[] = {
        {"CX"},
        {"ST"},
        {"SIM"},
        {"SIMP"},
        {"CAL"},
        {"MEC,EGREL"},
        {"MEC,LDAR"},
        {"MEC,AUD"},
    };

    inline const char* GetCommandName(CommandType cmd) {
        if (cmd >= CommandType::CX && cmd <= CommandType::AUD) {
            return CommandTable[static_cast<int>(cmd)];
        }
        return "UNKNOWN";
    }

    struct StateVector{
        float x;
        float y;
        float z;
        float vx;
        float vy;
        float vz;
        float q0;
        float q1;
        float q2;
        float q3;
    };

    struct Vector3{
        float x;
        float y;
        float z;
    };

    struct BarometerData {
        float pressure;
        float temperature;
        float altitude;
    };

    struct GPSData {
        float latitude;
        float longitude;
        float altitude;
        float time;
        int satellites;
    };
    
    struct IMUData {
        Vector3 gyroVel;
        Vector3 acceleration;
        Vector3 eulerAngles;
    };

    // One queue message can carry one FIFO drain from the IMU task.
    inline constexpr size_t IMU_BATCH_MAX_SAMPLES = 4;

    struct TimestampedIMUSample {
        // Sensor sample reconstructed from one accel+gyro FIFO pair.
        IMUData sample;
        // Approximate sample time in MCU microseconds, reconstructed at FIFO drain.
        uint32_t timestampMicros;
    };

    struct IMUBatch {
        // Number of valid entries populated in samples[].
        uint8_t count;
        // Nominal IMU cadence used to reconstruct per-sample timestamps.
        uint32_t nominalSamplePeriodMicros;
        // Ordered oldest-to-newest samples from a single FIFO drain.
        TimestampedIMUSample samples[IMU_BATCH_MAX_SAMPLES];
    };

    struct VoltageData {
        float voltage;
        float current;
        float power;
    };

    struct LidarData {
        float distanceMeters;
    };
    struct MagData {
        float magX;
        float magY;
        float magZ;
    };

    struct SensorSnapshot {
        BarometerData barometerData;
        IMUData imuData;
        GPSData gpsData;
        VoltageData voltageData;
        LidarData lidarData;
        MagData magData;
    };

    struct telemetry {  // This struct needs to be reordered for memory optimization
        float missionTime;
        int packetCount;
        char Mode;
        FlightState state;
        float baroAltitude;
        float temperature;
        float pressure;
        float voltage;
        float current;
        IMUData imuData;
        MagData magnetometer;
        float GPSTime;
        Vector3 GPSLoc;
        int GPSSats;
        char CMDEcho[32];
        StateVector stateVector;
    };

    template<int N>
    class Window{
    public:
        void push(float value){
            _buffer[idx % N] = value;
            idx++;
            if(count < N) count++;
        }

        std::optional<float> average() const {
            if(count == 0) return std::nullopt;
            float sum = 0;
            for(int i = 0; i < count; i++){
                sum += _buffer[i];
            }
            return sum / static_cast<float>(count);
        }

        int size() const {
            return count;
        }

        bool isDescending() const {
            if(count < 2) return false;
            int down = 0;
            for(int i = 1; i < count; i++){
                float prev = _buffer[(i - 1) % count];
                float curr = _buffer[i];
                if(curr < prev) down++;
            }
            return down > (count - 1) / 2;
        }

        bool isAscending() const {
            if(count < 2) return false;
            int up = 0;
            for(int i = 1; i < count; i++){
                float prev = _buffer[(i - 1) % count];
                float curr = _buffer[i];
                if(curr > prev) up++;
            }
            return up > (count - 1) / 2;
        }

        bool isStatic() const {
            if(count < 2) return false;
            int flat = 0;
            for(int i = 1; i < count; i++){
                float prev = _buffer[(i - 1) % count];
                float curr = _buffer[i];
                if(abs(curr - prev) < 0.1f) flat++;
            }
            return flat > (count - 1) / 2;
        }

        void clear() {
            idx = 0;
            count = 0;
            std::fill(std::begin(_buffer), std::end(_buffer), float{});
        }

    private:
        float _buffer[N];
        int idx = 0;
        int count = 0;
    };
    
    inline int BuildPacket(const telemetry& data, char* buffer, size_t bufferSize){ // packet format is placeholder
        // Simple packet builder that writes into a fixed buffer (no heap).
        return std::snprintf(
            buffer,
            bufferSize,
            "ID:%d,Time:%f,PacketCount:%d,Mode:%c,State:%s,BaroAlt:%f,Temp:%f,Pressure:%f,"
            "Voltage:%f,Current:%f,GyroX:%f,GyroY:%f,GyroZ:%f,AccX:%f,AccY:%f,AccZ:%f,GPSTime:%f,"
            "GPSLat:%f,GPSLon:%f,GPSAlt:%f,GPSSats:%d,CMDEcho:%s,MagX:%f,MagY:%f,MagZ:%f,PosX:%f,"
            "PosY:%f,PosZ:%f,VelX:%f,VelY:%f,VelZ:%f,QuatW:%f,QuatX:%f,QuatY:%f,QuatZ:%f",
            TEAM_ID,
            data.missionTime,
            data.packetCount,
            data.Mode,
            Common::GetFlightStateName(data.state),
            data.baroAltitude,
            data.temperature,
            data.pressure,
            data.voltage,
            data.current,
            data.imuData.gyroVel.x,
            data.imuData.gyroVel.y,
            data.imuData.gyroVel.z,
            data.imuData.acceleration.x,
            data.imuData.acceleration.y,
            data.imuData.acceleration.z,
            data.GPSTime,
            data.GPSLoc.x,
            data.GPSLoc.y,
            data.GPSLoc.z,
            data.GPSSats,
            data.CMDEcho,
            data.magnetometer.magX,
            data.magnetometer.magY,
            data.magnetometer.magZ,
            data.stateVector.x,
            data.stateVector.y,
            data.stateVector.z,
            data.stateVector.vx,
            data.stateVector.vy,
            data.stateVector.vz,
            data.stateVector.q0,
            data.stateVector.q1,
            data.stateVector.q2,
            data.stateVector.q3
        );
    }



    
};

#endif // COMMON_H
