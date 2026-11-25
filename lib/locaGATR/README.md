# locaGATR

The robot pose on the Brain, from one task. Inches and degrees.

```cpp
double x = localizer.getX();
locagatr::Pose p = localizer.getPose(); // x, y, heading, pitch, roll
```

Needs GATR2-Electronics next to this repo for the control board link
(`brain/locaGATR`).

## Setup

1. Season `Makefile`:

   ```make
   GATR2_LIBS := spectagatr locagatr
   GATR2_BRAIN_LIBS := locagatr
   ```

2. The robot in `src/robot.cpp` ([gatr2/robot.h](../gatr2/include/gatr2/robot.h)),
   what runs in `src/localization.cpp`, then:

   ```cpp
   locagatr::Localizer localizer(config());
   ```

3. In `initialize()`: `localizer.start();` The IMU calibrates for about 2 s,
   keep the robot still.

## Use

| Call | Returns |
|---|---|
| `getPose()` | x, y, heading, pitch, roll |
| `getX()`, `getY()` | Inches |
| `getHeading()` | Degrees counterclockwise from +x, 0 to 360 |
| `getPitch()`, `getRoll()` | Degrees |
| `setPose(x, y, heading)` | Sets the pose. Waits about one tick |
| `startLog()`, `stopLog()`, `isLogging()` | Every tick to a file on the microSD card, see [Log](#log) |
| `getStatus()` | Last tick: raw readings, measurements, estimate, what runs |
| `describe()` | The status as text |

Any task can call these. Field frame: +x right, +y up on the field diagram.
Robot frame: +x forward, +y left.

## Pipeline

Each tick, `rate_hz` times a second, the Localizer task
([localizer.cpp](pros/localizer.cpp)) calls `Pipeline::tick()`
([pipeline.cpp](src/pipeline.cpp)):

```
1. acquisition  Source[]     -> SensorMap  counts, motor degrees, IMU sample
2. processing   Processor[]  -> MotionMap  named measurements, inches and degrees
3. estimation   Estimator    -> Estimate   the published pose
```

`Config` ([config.h](include/locagatr/config.h)) lists what runs in each
stage. `enabled = false` turns an entry off without deleting it.

| Stage | Base | Implementations |
|---|---|---|
| Sources | [sources/source.h](include/locagatr/sources/source.h) | `VexImuSource`, `ControlBoardSource`, `MotorEncoderSource` |
| Control board decoders | [sources/decoders/record_decoder.h](include/locagatr/sources/decoders/record_decoder.h) | `EncoderCountDecoder` (default) |
| Processors | [processors/processor.h](include/locagatr/processors/processor.h) | `TrackingWheelProcessor`, `WheelPairProcessor`, `ImuProcessor`, `DriveMotorProcessor` |
| Estimator | [estimators/estimator.h](include/locagatr/estimators/estimator.h) | `OdometryEstimator` (default) |

Processors give measurements as totals since start, so a missed tick loses
nothing:

| Processor | Measurement |
|---|---|
| `TrackingWheelProcessor` | One per wheel, under its name: inches forward or lateral |
| `WheelPairProcessor` | `"wheel pair"`: yaw from two forward wheels |
| `ImuProcessor` | `"IMU"`: yaw, pitch, roll |
| `DriveMotorProcessor` | `"drive"`: forward, lateral for mecanum (reads short) |

`OdometryEstimator` takes one measurement per quantity:

```cpp
c.odometry.forward  = {"left"};  // several are averaged
c.odometry.lateral  = "back";
c.odometry.yaw      = "IMU";     // or "wheel pair"
c.odometry.attitude = "IMU";
```

## Status text

```
sources     VEX IMU: ok | control board: FAILED | motor encoders: ok
processors  tracking wheels: ok | wheel pair: off | IMU: ok | drive: ok
estimator   odometry: waiting for "back"
board       not streaming, 0 frames, 0 dropped, 0 unhandled records
log         writing, /usd/run007.csv, 412 rows, 0 dropped
motion      IMU yaw 91.2 pitch 0.4 roll -0.1 | drive fwd 12.00 lat 0.10
pose        x 12.30 y 0.40 heading 91.2
ticks       412, 0 overruns
```

`FAILED`: could not begin (bad port, missing config). `off`: disabled.
Don't print it while spectaGATR runs; both use the same serial output.

## Log

```cpp
void autonomous() {
    localizer.startLog(); // a new file: /usd/run001.csv, run002.csv, ...
    // run the route
    localizer.stopLog();  // returns once the file is complete
}
```

Anywhere after `start()`, from any task. `startLog()` returns false without
a microSD card, and `getStatus().log.error` says why; the robot runs as
usual either way. Stopping the program without `stopLog()` is fine: the file
is closed between writes, so only the last 0.5 s or so is lost.

One row per tick, the raw readings before processing plus the pose it
published:

| Columns | Holds |
|---|---|
| `t_us` | Tick time, Brain microseconds |
| `board_open`, `board_fresh`, `board_restarted`, `board_frames`, `board_dropped` | Control board link |
| `tw0` to `tw7` | Encoder counts by board connector |
| `m1` to `m21` | Motor degrees by Smart Port |
| `imu_rotation`, `imu_pitch`, `imu_roll`, `imu_gyro_*`, `imu_accel_*` | IMU sample |
| `x`, `y`, `heading`, `pitch`, `roll` | The published pose |
| `set_x`, `set_y`, `set_heading` | A `setPose()` that took effect before this tick |

An empty cell means no reading. Rows are written twice a second from a low
priority task, so the Localizer never waits on the card. If the card falls
behind for more than 2.5 s, rows are dropped and counted. At 100 Hz a two
minute run is about 3 MB.

## Faults

| Fault | Result |
|---|---|
| Board frames lost | Nothing lost, caught up on the next frame; counted in `dropped` |
| Board port fails to open | Retried about every 0.5 s; status says `port not open` |
| Board restarts | Totals carry on; motion during the restart is lost |
| Chosen measurement missing | x and y hold, then take the whole change. `estimate.missing` names it |
| Yaw missing | Heading holds |
| Wheel slip | Not caught. Needs an estimator that gates against the IMU |

## Tuning

| Setting | How |
|---|---|
| `counts_per_rev` | Spin the wheel 10 turns, divide the count change by 10 |
| `direction` | Push forward (left for a sideways wheel). Count goes down: -1 |
| `diameter` | Push a measured distance, scale until `getX()` matches |
| `offset` | Turn in place 10 times, adjust until x and y return |
| `drive` | Drive a measured distance, compare `"drive"` in the status |

The board streams on its own, so a tick never waits for it: it reads what has
arrived and keeps the newest frame. `overruns` counts slow ticks.

## Adding an implementation

Derive from the stage base above, in the same folder:
`sources/<what>_source.h`, `sources/decoders/<what>_decoder.h`,
`processors/<what>_processor.h` or `estimators/<what>_estimator.h`. Source in
`src/` (`pros/` if it calls PROS), a test in `tests/`, both in
`CMakeLists.txt`, the source in `lib.mk`. Then add it to `Config`.

## Host tests

From GATR2-Override (Windows: MSYS2 ucrt64 on PATH, `-G "MinGW Makefiles"`):

```sh
cmake -S lib/locaGATR -B build-locagatr
cmake --build build-locagatr --parallel
ctest --test-dir build-locagatr --output-on-failure
```

## Status

52 host tests, builds with PROS 4.2.2. Runs on the 2026-27 robot; the log is
not yet run on a Brain. Motor counts per
turn (1800, 900, 300 by cartridge) are from the PROS docs, unchecked.
