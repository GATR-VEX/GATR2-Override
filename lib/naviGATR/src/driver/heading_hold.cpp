// heading_hold.cpp

#include "navigatr/driver/heading_hold.h"

#include <algorithm>
#include <cmath>

#include "navigatr/types.h"

namespace navigatr
{

double HeadingHold::turn(double forward, double strafe, double turn, double heading,
                         uint32_t now_ms) {
    if (std::fabs(turn) > config_.deadband) {
        holding_       = false;
        turned_        = true;
        turn_released_ = now_ms;
        return turn;
    }
    // Let go of the turn stick: the robot keeps turning a moment, hold where it stops.
    if (turned_ && now_ms - turn_released_ < config_.settle_ms) {
        return 0;
    }
    turned_ = false;

    const bool moving = std::fabs(forward) > config_.deadband || std::fabs(strafe) > config_.deadband;
    if (!moving) {
        holding_ = false; // standing still: nothing to twist, and a push is not fought
        return 0;
    }
    if (!holding_) {
        holding_ = true;
        target_  = heading;
    }
    // Twisted counterclockwise past the target (error < 0): turn right, a positive stick.
    const double error = wrap180(target_ - heading);
    return std::clamp(-config_.kp * error, -config_.max_turn, config_.max_turn);
}

} // namespace navigatr
