# naviGATR

Autonomous routes and path following, using the pose from a locaGATR
Localizer. Inches and degrees.

```cpp
const navigatr::Route route = navigatr::Route({24, 12, 90})   // x, y, heading
    .bezier({36, 24}, {48, 48}, {60, 48}).facing(0)
    .line({60, 72})
    .at(0.5, [] { intake.move(127); })
    .turnTo(180)
    .wait(300);

localizer.setPose(route.startPose());
navigator.follow(route);
```

## Setup

1. locaGATR set up first.
2. Season `Makefile`: `GATR2_LIBS := spectagatr locagatr navigatr`.
3. Limits and gains in `src/navigation.cpp`, then:

   ```cpp
   navigatr::Navigator navigator(config(), localizer);
   ```

## Routes

| Call | Does |
|---|---|
| `Route({x, y, heading})` | Start pose |
| `path()` | The next segment starts a new move, so `next()` stops before it |
| `line({x, y})` | Straight to the point |
| `bezier(c1, c2, end)` | Cubic Bezier: two control points, then the end |
| `facing(heading)` | Last segment ends facing this way |
| `tangent()` | Last segment faces along the path |
| `at(fraction, action)` | Action at a fraction of the last segment, or at the start of a turn or wait |
| `turnTo(heading)` | Turn in place |
| `wait(ms)` | Stop |

Lines and curves in a row are one move, no stop between them. The heading
holds unless `facing()` or `tangent()` says otherwise. Actions run in the
navigator task, so keep them short.

## Running

| Call | Does |
|---|---|
| `follow(route)` | Runs it, blocks. True if every step settled |
| `start(route)` | Runs it, returns at once |
| `load(route)` | Plans it for `next()` |
| `next()` | Runs the next step (move, turn or wait), returns at once. False after the last |
| `wait()` | Until done |
| `waitUntil(fraction)` | Until that fraction of the driving distance, of the route or of the step |
| `cancel()` | Stops the route and the drive |
| `isRunning()`, `progress()` | |
| `outcome()` | Cancelled, steps timed out, first one |

```cpp
navigator.start(route);
navigator.waitUntil(0.5);
lift.score();  // slow work while it drives
navigator.wait();
```

One step at a time, for big actions with the robot stopped. Each move is the
lines and curves in a row; a step starts where the last one ended.

```cpp
localizer.setPose(route.startPose());
navigator.load(route);

navigator.next();          // first move
navigator.waitUntil(0.5);  // half of this move
lift.up();
navigator.wait();
score();                   // stopped

navigator.next();          // the turn
navigator.wait();
navigator.next();          // false once the route is done; nothing restarts
```

A route stops on `cancel()` or when the competition mode changes. A step
settles within `tolerances` for `settle_ms`, or gives up `timeout_ms` late;
the rest of the route still runs.

## Pipeline

The Navigator task ([navigator.cpp](pros/navigator.cpp)) calls
`Pipeline::run()` ([pipeline.cpp](src/pipeline.cpp)) once per route, or
`runStep()` once per `next()`. The whole route is planned first, then each
step runs:

```
1. path      segments -> Path
2. profile   Path -> Trajectory          speed and time per point
then every tick:
3. follow    pose + goal -> command      forward, left, turn
4. drive     command -> motors
```

| Stage | Base | Implementations |
|---|---|---|
| Path | [path.h](include/navigatr/path.h) | `samplePath` |
| Profile | [profiles/profile.h](include/navigatr/profiles/profile.h) | `TrapezoidProfile` (default): slower in tight curves and fast heading changes |
| Follower | [followers/follower.h](include/navigatr/followers/follower.h) | `PurePursuitFollower` (default), `TrajectoryFollower` |
| Drive | [drives/drive.h](include/navigatr/drives/drive.h) | `MecanumDrive` |

`Config` ([config.h](include/navigatr/config.h)) picks the implementations and
holds the limits, gains, tolerances and the shared `gatr2::Robot`.

### Followers

Each tick the follower gets a `Goal`: the trajectory point at time t, and for
a move the whole path and trajectory. Each takes what it needs.

| Follower | Follows | Robot falls behind |
|---|---|---|
| `PurePursuitFollower` | The point `lookahead` in further along the path than the robot, at the speed allowed there | Carries on from where it is |
| `TrajectoryFollower` | The trajectory point at t: its velocity plus PID on x, y, heading | Chases a target that keeps going, and cuts curves |

Both are for drives that strafe (mecanum). Pure pursuit cuts the inside of
curves by an amount set by `lookahead` (default 6 in), whatever the speed.
The trajectory follower is tighter while the robot keeps up and worse once it
does not. On the simulated robot, a quarter circle about a tile across:

| Robot speed | Pure pursuit, 6 in | Pure pursuit, 12 in | Trajectory |
|---|---|---|---|
| As commanded | 0.8 in off the path | 2.5 in | 0.3 in |
| 75% of it | 0.8 in | 2.5 in | 1.6 in |

Pure pursuit settles on the end pose with the trajectory follower's PID
within `lookahead` of the end, and for turn steps. Another lookahead, or the
trajectory follower:

```cpp
navigatr::PurePursuitFollower pure_pursuit({4});  // PurePursuitConfig: lookahead, search
navigatr::TrajectoryFollower  trajectory;
c.follower = &pure_pursuit;                       // or &trajectory
```

## Tuning

Start around 24 in/s with the localizer already checked.

| Setting | How |
|---|---|
| `Robot::drive` | Measure rpm, gear ratio, wheel diameter, track width, wheelbase. `kv` comes from these |
| `feedforward.ks` | Raise until the robot just moves |
| `feedforward.kv` | Leave at 0 unless it runs consistently fast or slow |
| `gains.kp`, `gains.heading_kp` | Raise until it holds the path without wobble |
| `lookahead` (pure pursuit) | Wobbles side to side: raise. Cuts curves: lower |
| `limits` | Raise once it follows cleanly |

## Driver control

Driver control goes through the same `Drive` as auton, so the ports, wheel
mixing and scaling live in one place:

```
sticks -> HeadingHold (fills in turn) -> stickSpeeds -> Drive -> motors
route  -> follower                                   -> Drive -> motors
```

```cpp
const double dir        = navigatr::deadband(master.get_analog(ANALOG_LEFT_Y));
const double strafe     = navigatr::deadband(master.get_analog(ANALOG_LEFT_X));
const double stick_turn = navigatr::deadband(master.get_analog(ANALOG_RIGHT_X));
const double turn = hold.turn(dir, strafe, stick_turn, localizer.getHeading(), pros::millis());
drive.apply(navigatr::stickSpeeds({dir, strafe, turn}, drive.top()));
```

`stickSpeeds` ([driver/sticks.h](include/navigatr/driver/sticks.h)) maps a
full stick to the drive's top speed, `Drive::top()`, so full stick is full
voltage. Call `drive.begin(config)` before the first `apply()`; the
navigator only begins it on its first route.

### Heading hold

Mecanum twists when it strafes. `HeadingHold`
([driver/heading_hold.h](include/navigatr/driver/heading_hold.h)) holds the
heading with the IMU while the turn stick is centered. Stick units in and
out, the turn positive to the right. It holds only while the robot drives or
strafes; standing still it lets go, so a push is not fought. After the turn
stick is let go it waits `settle_ms` and holds where the robot stopped
turning. Call `reset()` after an auton route.

| Setting | How |
|---|---|
| `kp` (2) | Strafe full speed. Still twists: raise. Wobbles: lower |
| `max_turn` (50) | The most it ever adds to the turn |
| `tolerance` (1 deg) | Buzzes while holding: raise |
| `settle_ms` (250) | Snaps back after a turn: raise |

## Host tests

From GATR2-Override (Windows: MSYS2 ucrt64 on PATH, `-G "MinGW Makefiles"`):

```sh
cmake -S lib/naviGATR -B build-navigatr
cmake --build build-navigatr --parallel
ctest --test-dir build-navigatr --output-on-failure
```

## Status

62 host tests, including whole routes and single steps on a simulated robot.
Driver control through `Drive`, heading hold and pure pursuit not yet run on
a robot.
Builds with PROS 4.2.2. Not run on a robot; the navigator task (start, load,
next, wait, cancel) has not run at all.
