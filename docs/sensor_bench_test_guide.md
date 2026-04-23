# Sensor Bench Test Guide

Yes this is AI generated as an engineer i don't know how to write sentences - Kyle

This guide is for running the IMU, magnetometer, and LiDAR bench firmware one sensor at a time on the Teensy 4.1.

Use this guide when:
- the production code changes need hardware validation
- the sensor hardware is physically available on the PM's bench
- the goal is to confirm basic communication and expected sensor behavior, not full mission integration

This guide assumes:
- the repository is already cloned locally
- PlatformIO is installed and usable from the terminal
- the Teensy 4.1 can be connected to the computer by USB
- only the target sensor for the current test is connected to the MCU test bench
- the sensor is already wired correctly for power, ground, and I2C

If any of those assumptions are false, stop and ask for help before testing.

## Safety and Scope

- Only test one sensor at a time.
- Do not hot-swap sensor wiring while the board is powered.
- Unplug USB power before moving wiring.
- These tests only check communication and obvious data behavior.
- A passing bench test does not mean the full flight software is ready.

## Common Setup

1. Connect the Teensy 4.1 to the computer by USB.
2. Connect only the sensor being tested.
3. Open a terminal in the repository root.
4. Confirm the Teensy is detected by the OS.
5. Use the commands in this guide exactly.

All bench targets use `115200` baud for serial output.

## General Command Pattern

Replace `<env>` with one of:
- `imu_bench`
- `mag_bench`
- `lidar_bench`

Build:

```bash
pio run -e <env>
```

Flash:

```bash
pio run -e <env> -t upload
```

Open serial monitor:

```bash
pio device monitor -b 115200
```

If `pio device monitor` attaches to the wrong device, ask for help rather than guessing.

## What Evidence to Capture

For every sensor test:
- take a screenshot or copy/paste of the terminal output
- capture at least one successful startup line
- capture at least 10 seconds of live sensor output
- if a test fails, capture the failure output exactly as shown
- record what physical action was taken during the test
  - for example: "rotated board by hand", "moved target from 0.5 m to 1.5 m", "left board stationary"

## IMU Test

### Goal

Confirm that:
- the IMU initializes
- direct accel/gyro reads are live
- FIFO reads are live
- FIFO diagnostics do not show obvious faults during normal use

### Commands

```bash
pio run -e imu_bench
pio run -e imu_bench -t upload
pio device monitor -b 115200
```

### Expected Startup Output

You should see:

```text
imu_bench: starting
imu_bench: init=OK
```

If you see `init=FAIL`, stop and record the failure.

### Expected Live Output

You will see repeated lines like:

```text
imu_bench: direct gyro=... accel=... fifo_count=... fifo=[...] diag={overrun=...,nearFull=...,unexpectedTag=...,truncated=...}
```

### Pass Criteria

- `imu_bench: init=OK` appears once after boot.
- `direct gyro=` and `accel=` values are not frozen at obvious nonsense.
- When the board is moved or rotated, the direct gyro and accel values change.
- `fifo_count=` is usually greater than `0` and often between `1` and `4`.
- The `fifo=[...]` section appears when samples are present.
- The printed `t=` values inside one FIFO line move forward and do not go backward.
- `diag={...}` counters stay at `0` or at least do not climb rapidly during normal testing.

### Failure Signs

- `init=FAIL`
- `fifo_count=0` forever even while the board is moving
- direct gyro and accel values never change when the board is moved
- `unexpectedTag`, `truncated`, or `overrun` counters keep increasing
- printed timestamps inside one FIFO line move backward

### Physical Actions to Try

- Leave the board flat and still for 5 seconds.
- Rotate the board slowly by hand.
- Tilt the board through several orientations.
- Return it to rest and observe whether values stabilize.

## Magnetometer Test

### Goal

Confirm that:
- the magnetometer initializes
- XYZ magnetic field readings are live
- the driver diagnostics do not show read failures during normal use

### Commands

```bash
pio run -e mag_bench
pio run -e mag_bench -t upload
pio device monitor -b 115200
```

### Expected Startup Output

You should see:

```text
mag_bench: starting
mag_bench: init=OK
```

If you see `init=FAIL`, stop and record the failure.

### Expected Live Output

You will see repeated lines like:

```text
mag_bench: xyz=... diag={initFail=...,readFail=...,stale=...}
```

### Pass Criteria

- `mag_bench: init=OK` appears after boot.
- `xyz=` values change when the board is rotated in different directions.
- `initFail=0`
- `readFail=0`
- `stale=0` during normal testing

### Failure Signs

- `init=FAIL`
- XYZ values stay fixed while the board is rotated
- `readFail` keeps increasing
- `stale` keeps increasing

### Physical Actions to Try

- Rotate the board slowly around different axes.
- Turn the board 90 degrees at a time.
- Move it away from large metal objects if readings look unstable.

## LiDAR Test

### Goal

Confirm that:
- the LiDAR initializes
- distance readings are live
- validity and status fields behave correctly

### Commands

```bash
pio run -e lidar_bench
pio run -e lidar_bench -t upload
pio device monitor -b 115200
```

### Expected Startup Output

You should see:

```text
lidar_bench: starting
lidar_bench: init=OK
```

If you see `init=FAIL`, stop and record the failure.

### Expected Live Output

You will see repeated lines like:

```text
lidar_bench: distance_m=... valid=... status=... failures=...
```

### Pass Criteria

- `lidar_bench: init=OK` appears after boot.
- `valid=true`
- `status=Ok`
- `failures=0` during normal reads
- `distance_m=` changes when the target distance changes

### Failure Signs

- `init=FAIL`
- `valid=false` most or all of the time
- `status=NoAck`
- `status=InvalidRange`
- `failures` keeps increasing
- distance never changes when the target is moved

### Physical Actions to Try

- Place a flat target at a short known distance.
- Move the target farther away.
- Remove the target briefly and observe whether validity/status changes.

## Voltage Sensor Test (INA260)

### Goal

Confirm that:
- the INA260 initializes over I2C (WHO_AM_I check passes)
- voltage, current, and power readings are live and non-zero when the power rail is connected
- driver diagnostics show no read failures during normal use

### Prerequisites

Before running this test:
- Confirm the INA260 I2C address strapping (A0/A1) matches the `kIna260Address` constant in `VoltageSensor.cpp` (default 0x40 = A0=A1=GND).
- Confirm the I2C bus wiring matches the Wire instance used in the driver (default: Wire).
- Connect the INA260 to a live power rail so current and voltage are measurable.

### Commands

```bash
pio run -e voltage_bench
pio run -e voltage_bench -t upload
pio device monitor -b 115200
```

### Expected Startup Output

```text
voltage_bench: starting
voltage_bench: init=OK
```

If you see `init=FAIL`, the WHO_AM_I check failed. Check I2C wiring, address strapping, and bus assignment.

### Expected Live Output

```text
voltage_bench: voltage_mV=... current_mA=... power_mW=... diag={initFail=0,readFail=0,stale=0}
```

### Pass Criteria

- `init=OK` appears after boot.
- `voltage_mV=` is non-zero and close to the known supply voltage.
- `current_mA=` is non-zero when a load is present.
- `power_mW=` is consistent with voltage × current.
- `diag={initFail=0,readFail=0,stale=0}` throughout.

### Failure Signs

- `init=FAIL` — I2C NAK or wrong Manufacturer ID; check address and bus.
- All values frozen at 0.00 — likely `initialized_` is false; check init output.
- `readFail` or `stale` counts increasing — I2C read failures during operation.
- Values look plausible but obviously wrong (e.g., voltage reads ~10x actual) — big-endian byte order bug; must not occur since the driver uses `(raw[0] << 8) | raw[1]`.

### Physical Actions to Try

- Note the voltage with no load connected.
- Connect a known load and observe `current_mA` increase.
- Disconnect I2C lines mid-run and observe `readFail` and `stale` counts increase.

---

## Audio Beacon Test (CPI-3116-3-100T)

### Goal

Confirm that:
- the AudioBeacon initializes (GPIO pin configured)
- the buzzer activates audibly when triggered
- the driver toggles on/off at 1 Hz

### Prerequisites

Before running this test:
- Confirm `BUZZER_PIN` in `Common.h` matches the actual GPIO pin on the PCB.
- Confirm a transistor buffer is present if required (CPI-3116-3-100T draws 30 mA; Teensy GPIO max is ~10 mA).
- Do NOT power the buzzer circuit before confirming the buffer situation.

### Commands

```bash
pio run -e audio_beacon_bench
pio run -e audio_beacon_bench -t upload
pio device monitor -b 115200
```

### Expected Startup Output

```text
audio_beacon_bench: starting
audio_beacon_bench: init=OK
```

### Expected Live Output

```text
audio_beacon_bench: state=ON diag={initFail=0}
audio_beacon_bench: state=OFF diag={initFail=0}
```

The state alternates every 1 second.

### Pass Criteria

- `init=OK` appears after boot.
- Serial output alternates `state=ON` and `state=OFF` at 1 Hz.
- The buzzer is audibly heard when `state=ON` is printed.
- `initFail=0` throughout.

### Failure Signs

- `init=FAIL`
- State toggles in serial output but no sound is heard — GPIO may not be reaching the buzzer; check wiring, buffer, and pin number.
- State does not toggle — bench firmware issue.

### Physical Actions to Try

- Hold a multimeter on the buzzer GPIO pin and confirm it toggles between LOW and HIGH at 1 Hz.
- Listen for the audible tone when `state=ON`.

---

## Camera Bench Test (ESP32-CAM trigger)

### Goal

Confirm that:
- the Camera driver initializes (GPIO pin configured)
- the trigger GPIO goes HIGH ~1.5 s after boot and stays HIGH
- the GPIO level can be confirmed with a multimeter

### Prerequisites

Before running this test:
- Confirm `CAMERA_PIN` in `Common.h` matches the actual Teensy GPIO pin connected to the ESP32-CAM boards.
- Confirm the Teensy 3.3 V output connects directly to the ESP32-CAM GPIO input (no level shifter needed for 3.3 V).
- The ESP32-CAM boards and their 5 V power rail are outside the scope of this bench test. The bench only verifies the Teensy GPIO signal.

### Commands

```bash
pio run -e camera_bench
pio run -e camera_bench -t upload
pio device monitor -b 115200
```

### Expected Startup Output

```text
camera_bench: starting
camera_bench: init=OK
camera_bench: trigger=HIGH both ESP32-CAMs should begin recording
```

### Expected Live Output

```text
camera_bench: capturing=true diag={startCount=1,stopCount=0}
```

The line repeats every 500 ms. `capturing=true` and `startCount=1` should remain constant — the trigger is not cycled.

### Pass Criteria

- `init=OK` appears after boot.
- `trigger=HIGH` line appears ~1.5 s after `init=OK`.
- `capturing=true` and `startCount=1` on every subsequent line.
- A multimeter on the camera GPIO pin reads HIGH and stays HIGH.

### Failure Signs

- `init=FAIL`
- `capturing=false` or `startCount=0` — `StartCapture()` was not called or failed.
- Multimeter reads LOW after trigger line — GPIO pin number is wrong.

### Physical Actions to Try

- Place a multimeter probe on the camera trigger GPIO pin.
- Confirm pin reads LOW before the trigger line, then HIGH after.
- If an ESP32-CAM board is connected and flashed, observe its SD write indicator.

---

## Radio Bench Test (XBee transparent mode)

### Goal

Confirm that:
- the Radio driver initializes (serial port opened at 115200)
- test packets are transmitted without TX errors
- received bytes are printed when the XBee echoes or a remote node replies

### Prerequisites

Before running this test:
- Confirm `kXbeeSerial` in `Radio.cpp` matches the Teensy hardware serial port the XBee is physically wired to (default: `Serial1`; update if different).
- Confirm the XBee module has been pre-configured to 115200 baud. The factory default is 9600 baud. If the XBee is still at 9600, the bench will appear to succeed but no data will pass.
- For loopback testing, physically connect the Teensy TX pin to the Teensy RX pin for that serial port to echo sent bytes back.

### Commands

```bash
pio run -e radio_bench
pio run -e radio_bench -t upload
pio device monitor -b 115200
```

### Expected Startup Output

```text
radio_bench: starting
radio_bench: xbee_baud=115200
radio_bench: init=OK
```

### Expected Live Output

Without a remote node or loopback:
```text
radio_bench: tx=OK diag={sent=18,recv=0,txErr=0}
radio_bench: tx=OK diag={sent=36,recv=0,txErr=0}
```

With TX→RX loopback wired:
```text
radio_bench: rx=radio_bench: test
radio_bench: tx=OK diag={sent=18,recv=18,txErr=0}
```

### Pass Criteria

- `init=OK` appears after boot.
- `tx=OK` on every transmission line.
- `sent=` count increases by 18 each second (length of the test packet).
- `txErr=0` throughout.
- With loopback: `recv=` matches `sent=` and RX lines appear.

### Failure Signs

- `init=FAIL` (should not happen; `HardwareSerial::begin()` always succeeds)
- `tx=FAIL` — TX buffer overflow; should not occur at 1 Hz with 18-byte packets
- `txErr` increasing — same cause
- `sent=` increases but `recv=0` with loopback wired — XBee baud rate mismatch or wrong serial port
- No output at all — wrong serial port assigned in `kXbeeSerial`

### Physical Actions to Try

- First run without loopback; confirm `tx=OK` and `sent=` increases.
- Wire TX→RX on the XBee serial port and confirm received bytes appear.
- If an XBee is connected and paired with a remote node, confirm the remote node receives the test packet.

---

## Barometer Test (BMP390)

### Goal

Confirm that:
- the BMP390 initializes over I2C (CHIP_ID check passes)
- pressure and temperature readings are live and plausible
- driver diagnostics show no failures during normal use

### Prerequisites

Before running this test:
- Confirm the BMP390 SDO pin strapping matches the I2C address probed by the driver (SDO=GND → 0x76, SDO=VDD → 0x77; the driver tries both automatically).
- Confirm the I2C bus wiring matches the Wire instance used in the driver (default: Wire).

### Commands

```bash
pio run -e baro_bench
pio run -e baro_bench -t upload
pio device monitor -b 115200
```

### Expected Startup Output

```text
baro_bench: starting
baro_bench: init=OK
```

If you see `init=FAIL`, the CHIP_ID check failed or a configuration write failed. Check I2C wiring and SDO strapping.

### Expected Live Output

```text
baro_bench: pressure_hPa=... temp_C=... alt_m=0.00 diag={initFail=0,readFail=0,stale=0}
```

`alt_m` will always read `0.00` — altitude zeroing is outside the driver.

### Pass Criteria

- `init=OK` appears after boot.
- `pressure_hPa=` is close to the local atmospheric pressure (~1013 hPa at sea level, lower at altitude).
- `temp_C=` is a plausible room temperature (roughly 15–35 °C indoors).
- `diag={initFail=0,readFail=0,stale=0}` throughout.

### Failure Signs

- `init=FAIL` — CHIP_ID mismatch or config write failed; check wiring and address strapping.
- `pressure_hPa=0.00` and `temp_C=0.00` frozen — likely `initialized_` is false; check init output.
- `readFail` or `stale` counts increasing — I2C read failures during operation.

### Physical Actions to Try

- Note the pressure reading at rest indoors.
- Cup your hand over the sensor briefly and observe whether temperature responds.
- Move the sensor to a different elevation if possible and confirm pressure changes.

---

## GPS Bench Test (Teseo I2C / NMEA)

### Goal

Confirm that:
- the GPS driver initializes over I2C (bytes-available register responds)
- NMEA sentences are parsed without read failures
- latitude, longitude, and satellite count update when a fix is acquired outdoors

### Prerequisites

Before running this test:
- Confirm the GPS module I2C address matches `kGpsI2cAddress` (default 0x42).
- Confirm the I2C bus wiring matches the Wire instance used in the driver (default: Wire).
- Note: a GPS fix requires outdoor line-of-sight to satellites. Indoors, `lat=0.000000`, `lon=0.000000`, and `sats=0` are expected and are not failures.

### Commands

```bash
pio run -e gps_bench
pio run -e gps_bench -t upload
pio device monitor -b 115200
```

### Expected Startup Output

```text
gps_bench: starting
gps_bench: init=OK
```

If you see `init=FAIL`, the bytes-available register read failed. Check I2C wiring and address.

### Expected Live Output

Indoors (no fix):
```text
gps_bench: lat=0.000000 lon=0.000000 alt_m=0.00 sats=0 diag={initFail=0,readFail=0,stale=0}
```

Outdoors with fix (may take several minutes):
```text
gps_bench: lat=... lon=... alt_m=... sats=... diag={initFail=0,readFail=0,stale=0}
```

### Pass Criteria

- `init=OK` appears after boot.
- `diag={initFail=0,readFail=0,stale=0}` throughout.
- Outdoors: `sats=` is greater than 0 and `lat=`/`lon=` are non-zero and consistent with the known location.

### Failure Signs

- `init=FAIL` — I2C NAK on bytes-available register; check wiring and address.
- `readFail` keeps increasing — I2C read failures during NMEA polling.
- `stale` keeps increasing rapidly — not initialized; check init output.
- `sats=0` and `lat=0.0` outdoors after 5+ minutes — possible antenna problem or wrong module.

### Physical Actions to Try

- Run indoors first to confirm `init=OK` and `readFail=0`.
- Take the board outdoors with clear sky view.
- Wait up to 5 minutes for first fix (cold start).
- Confirm `sats=` increases and `lat=`/`lon=` become non-zero.

---

## What to Send Back After Testing

For each sensor:
- whether build succeeded
- whether upload succeeded
- whether `init=OK` or `init=FAIL`
- 10 seconds of serial output
- a short note describing what physical motion or target change was performed
- whether the result should be considered pass or fail based on the criteria above

For the voltage sensor also include:
- the known supply voltage and whether `voltage_mV=` is consistent with it

For the audio beacon also include:
- whether the buzzer was audible when `state=ON`

For the camera also include:
- the multimeter reading on the trigger GPIO pin before and after the trigger line
- whether the ESP32-CAM was connected and whether recording appeared to start

For the radio also include:
- whether loopback was wired and whether `recv=` matched `sent=`
- the XBee baud rate pre-configuration status (confirmed 115200 or not yet changed)

For the barometer also include:
- the `pressure_hPa=` reading and whether it is consistent with the known local atmospheric pressure
- the `temp_C=` reading and whether it is consistent with the room temperature

For the GPS also include:
- whether the test was run indoors or outdoors
- if outdoors: how long until first fix, final `sats=` count, and whether `lat=`/`lon=` match the known location

## When to Stop and Ask for Help

Stop and ask for help if:
- PlatformIO commands do not run
- upload fails repeatedly
- the serial monitor shows nothing at all
- the wrong sensor may be wired
- you are unsure whether a result is pass or fail
- the board or sensor becomes hot, unstable, or disconnects repeatedly
