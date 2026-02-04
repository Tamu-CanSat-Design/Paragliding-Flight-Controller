# Hardware Layer Guide

This folder is the hardware abstraction layer (HAL). It keeps sensor and actuator
code organized and makes it easy to swap a device without touching the rest of
the flight software.

## Layout

- `include/interfaces/` contains pure interfaces (what a sensor *does*).
  - Names use `I*` (example: `IImu` lives in `interfaces/IMU.h`).
- `include/drivers/` contains concrete device drivers (what a sensor *is*).
  - Names use the sensor type (example: `IMU` in `drivers/Imu.h`).
- `include/drivers/sim/` and `src/drivers/sim/` contain simulated drivers.
  - Names use `Simulated<sensor>` (example: `SimulatedImu`).
- `src/drivers/` contains the implementation `.cpp` files for drivers.

## Naming Conventions

- Prefer **sensor role** names over part numbers.
  - Use `IMU`, `GPS`, `Lidar`, `Barometer`, `Radio`, `VoltageSensor`, etc.
- If multiple devices perform the same role, use **function** naming.
  - Example: `ReleaseServo` vs `SteeringServo`.
- Interfaces always use `I*` to avoid collisions with driver classes.
  - Example: `IImu` vs `IMU`.

## Examples

Example IMU files live alongside the real code so new contributors can copy
the pattern in the correct place:

- Interface example: `lib/Hardware/include/interfaces/ExampleImu.h`
- Driver example: `lib/Hardware/include/drivers/ExampleImu.h` and `lib/Hardware/src/drivers/ExampleImu.cpp`
- Simulation example: `lib/Hardware/include/drivers/sim/SimulatedExampleImu.h` and `lib/Hardware/src/drivers/sim/SimulatedExampleImu.cpp`

## Driver Checklist

- Add a driver header in `include/drivers/`.
- Add an implementation in `src/drivers/`.
- Keep device-specific details inside the driver.
- Only expose generic data structures to the rest of the system.
