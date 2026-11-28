// sticks.h
// Driver control: controller sticks to a Drive command, full stick = the
// drive's top speed. Portable.
//
//   const double dir = navigatr::deadband(master.get_analog(ANALOG_LEFT_Y));
//   drive.apply(navigatr::stickSpeeds({dir, strafe, turn}, drive.top()));

#pragma once
#include <cmath>

#include "navigatr/types.h"

namespace navigatr
{

// -127 to 127.
struct Sticks {
    double forward = 0;
    double strafe  = 0; // positive right
    double turn    = 0; // positive right, clockwise
};

// 0 inside the band, so a stick that rests slightly off center does not creep the robot.
inline double deadband(double stick, double band = 5) { return std::fabs(stick) < band ? 0 : stick; }

ChassisSpeeds stickSpeeds(const Sticks& sticks, const ChassisSpeeds& top);

} // namespace navigatr
