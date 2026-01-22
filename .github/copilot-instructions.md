# Copilot / AI assistant instructions — Flight-Software-2026

Purpose: give an AI coding agent immediate, actionable context to be productive in this embedded PlatformIO project.

Big picture
- This is an embedded flight-software project built with PlatformIO (PlatformIO project file: `platformio.ini`).
- Runtime: Arduino-style `setup()`/`loop()` entry in [src/main.cpp](src/main.cpp).
- High-level flow: sensors -> hardware manager libraries -> RTOS-style Tasks in `include/Tasks` and `src/Tasks` -> state logic in `include/States`.

Key components & where to look
- Board/project config: `platformio.ini` (build environments and upload targets).
- Entry point: [src/main.cpp](src/main.cpp).
- Public headers (project-wide API): `include/` (examples: [include/HardwareManager.h](include/HardwareManager.h), [include/Common.h](include/Common.h)).
- State machine: [include/States/States.h](include/States/States.h) — describes mission states (Ascent, Descent, PayloadRelease, Recovery).
- Tasks and scheduling: `include/Tasks` and `include/States` — look at `Tasks/*.cpp` for task patterns (message queues, mutexes).
- Private libraries: `lib/` holds hardware-specific libraries compiled as static libs (example: `lib/Hardware/src/BMP390.cpp`).

Project-specific conventions & idioms
- Headers live in `include/` and may include implementation files in an unusual pattern (for example `HardwareManager.h` includes `BMP390.cpp`). Be careful: some components mix declarations and inline includes — prefer following existing style when modifying.
- Use `Common::telemetry` (defined in `include/Common.h`) as the shared telemetry packet structure; many modules update or read this.
- Inter-task comms use FreeRTOS/PlatformIO-style queues and mutexes (initialize in `setup()`; see `include/Tasks` files for usage examples).

Build / test / debug workflows (concrete commands)
- Build: `pio run` or `platformio run`
- Build + upload to default env: `pio run -t upload`
- List environments: `platformio run -e <env> --target compile` (or open `platformio.ini`).
- Monitor serial: `pio device monitor -b 115200` (adjust baud per `Serial.begin` in `src/main.cpp`).

Code patterns & examples to follow
- Hardware manager pattern: `include/HardwareManager.h` exposes `Initialize()`, `ReadSensors()`, `UpdateTelemetry()` — follow this separation when adding drivers.
- State functions in `include/States/States.h` are standalone functions (no class); implement corresponding logic in `include/States/*.cpp` or `src/States` while keeping signatures stable.
- Libraries in `lib/Hardware` follow a `include/` + `src/` split; new hardware should follow the same layout and, if needed, add `library.json` for custom build flags.

Integration points & external dependencies
- PlatformIO toolchain (build/upload/monitor).
- Hardware drivers live under `lib/Hardware` and `include/Hardware` — cross-compilation and I2C/SPI interfaces are abstracted inside those libs.
- Persistence/parameters: EEPROM usage indicated in `include/HardwareManager.h` — search for `EEPROM` usage when changing persistent config.

What a helpful PR from an AI assistant looks like
- Make minimal, focused changes: update a single driver or state implementation per PR.
- Preserve platform-specific build settings in `platformio.ini` and avoid changing board/envs unless requested.
- Include example serial logs or a short test plan in PR description (how to run `pio device monitor` and what output to expect).

If anything is unclear
- Ask for the target board/environment to use for builds (the `platformio.ini` may list multiple `env` entries).
- When adding drivers, request a hardware abstraction preference (simulate via `lib/Hardware/Simulated*` if available).

End
