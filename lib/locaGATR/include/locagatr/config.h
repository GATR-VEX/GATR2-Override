// config.h
// What the Localizer runs, and the robot it runs on. Portable.

#pragma once
#include <cstdint>
#include <vector>

#include "gatr2/robot.h"
#include "locagatr/types.h"

namespace locagatr
{

using gatr2::Axis;
using gatr2::DriveMotors;
using gatr2::Robot;
using gatr2::TrackingWheel;
using gatr2::WheelPair;

class Source;
class Processor;
class Estimator;

// Measurement names for OdometryEstimator. Several forward ones are averaged;
// null is not tracked.
struct OdometrySources {
    std::vector<const char*> forward;            // e.g. {"left", "right"} or {"drive"}
    const char*              lateral  = nullptr; // e.g. "back"
    const char*              yaw      = nullptr; // e.g. "IMU" or "wheel pair"
    const char*              attitude = nullptr; // pitch and roll, e.g. "IMU"
};

template <typename T> struct Enabled {
    T*   implementation = nullptr;
    bool enabled        = true; // false: never begun, never run
};

struct Config {
    uint32_t                        rate_hz = 100; // ticks per second
    Pose                            start_pose;
    std::vector<Enabled<Source>>    sources;
    std::vector<Enabled<Processor>> processors;
    Estimator*                      estimator = nullptr; // null: OdometryEstimator
    OdometrySources                 odometry;
    Robot                           robot;
    bool                            log = false; // every tick to the microSD card, log/run_log.h
};

} // namespace locagatr
