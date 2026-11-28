// sticks.cpp

#include "navigatr/driver/sticks.h"

namespace navigatr
{

ChassisSpeeds stickSpeeds(const Sticks& sticks, const ChassisSpeeds& top) {
    constexpr double kFull = 127;
    ChassisSpeeds    speeds;
    speeds.forward = sticks.forward / kFull * top.forward;
    speeds.left    = -sticks.strafe / kFull * top.left; // stick right, robot left negative
    speeds.turn    = -sticks.turn / kFull * top.turn;   // stick right turns clockwise
    return speeds;
}

} // namespace navigatr
