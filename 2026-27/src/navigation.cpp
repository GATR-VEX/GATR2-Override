// navigation.cpp
// Navigator limits, gains and tolerances. Inches, degrees.
// CHECK: first guess, tune it.

#include "navigation.h"

#include "localization.h"
#include "robot.h"

navigatr::MecanumDrive drive;

namespace
{

navigatr::Config config() {
    navigatr::Config c;
    c.rate_hz = 100;

    c.limits.max_speed         = 36;  // in/s. CHECK: start slow
    c.limits.max_accel         = 48;  // in/s^2. CHECK
    c.limits.max_decel         = 48;  // in/s^2. CHECK
    c.limits.max_lateral_accel = 48;  // in/s^2. CHECK
    c.limits.max_turn_rate     = 180; // deg/s. CHECK

    c.gains.kp         = 3; // CHECK
    c.gains.heading_kp = 4; // CHECK

    c.tolerances.position   = 1.0;
    c.tolerances.heading    = 2.0;
    c.tolerances.settle_ms  = 100;
    c.tolerances.timeout_ms = 1500;

    c.robot          = robot();
    c.feedforward.ks = 500; // mV. CHECK: least that moves the robot

    c.drive = &drive;
    return c;
}

} // namespace

navigatr::Navigator navigator(config(), localizer);

bool startDrive() { return drive.begin(config()); }
