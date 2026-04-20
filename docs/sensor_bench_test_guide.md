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

## What to Send Back After Testing

For each sensor:
- whether build succeeded
- whether upload succeeded
- whether `init=OK` or `init=FAIL`
- 10 seconds of serial output
- a short note describing what physical motion or target change was performed
- whether the result should be considered pass or fail based on the criteria above

## When to Stop and Ask for Help

Stop and ask for help if:
- PlatformIO commands do not run
- upload fails repeatedly
- the serial monitor shows nothing at all
- the wrong sensor may be wired
- you are unsure whether a result is pass or fail
- the board or sensor becomes hot, unstable, or disconnects repeatedly
