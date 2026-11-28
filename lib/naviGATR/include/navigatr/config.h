// config.h
// What the Navigator runs, its limits and gains, and the robot. Portable.

#pragma once
#include <cstdint>

#include "gatr2/robot.h"

namespace navigatr
{

class Profile;
class Follower;
class Drive;

struct Limits {
    double max_speed         = 48;  // in/s
    double max_accel         = 60;  // in/s^2
    double max_decel         = 60;  // in/s^2
    double max_lateral_accel = 60;  // in/s^2, slows the robot in tight curves
    double max_turn_rate     = 180; // deg/s
};

struct Gains {
    double kp         = 3; // in/s per in of position error
    double ki         = 0;
    double kd         = 0;
    double heading_kp = 4; // deg/s per deg of heading error
    double heading_ki = 0;
    double heading_kd = 0;
};

// A step is done within these for settle_ms, or gives up timeout_ms late.
struct Tolerances {
    double   position   = 1.0; // in
    double   heading    = 2.0; // deg
    uint32_t settle_ms  = 100;
    uint32_t timeout_ms = 1500;
};

// Wheel speed to motor millivolts.
struct Feedforward {
    // mV per in/s of wheel speed. 0: from Robot::drive.
    double kv     = 0;
    double ks     = 500; // mV to get a wheel moving
    double max_mv = 12000;
};

struct Config {
    uint32_t     rate_hz = 100; // follower updates per second
    Limits       limits;
    Gains        gains;
    Tolerances   tolerances;
    Feedforward  feedforward;
    gatr2::Robot robot;              // the same one the localizer gets; the drive reads Robot::drive
    Profile*     profile  = nullptr; // null: TrapezoidProfile
    Follower*    follower = nullptr; // null: PurePursuitFollower
    Drive*       drive    = nullptr; // required, e.g. MecanumDrive
};

} // namespace navigatr
