// trajectory_follower.cpp

#include "navigatr/followers/trajectory_follower.h"

#include <algorithm>
#include <cmath>

namespace navigatr
{

bool TrajectoryFollower::begin(const Config& config) {
    const Gains& g = config.gains;
    x_             = Pid(g.kp, g.ki, g.kd);
    y_             = Pid(g.kp, g.ki, g.kd);
    heading_       = Pid(g.heading_kp, g.heading_ki, g.heading_kd);
    max_turn_rate_ = config.limits.max_turn_rate;
    return max_turn_rate_ > 0;
}

void TrajectoryFollower::reset() {
    x_.reset();
    y_.reset();
    heading_.reset();
}

ChassisSpeeds TrajectoryFollower::update(const Pose& pose, const Goal& goal, double dt) {
    const Target& target = goal.target;
    const double  vx     = target.vx + x_.update(target.x - pose.x, dt);
    const double  vy     = target.vy + y_.update(target.y - pose.y, dt);
    const double  turn =
        target.turn_rate + heading_.update(wrap180(target.heading - pose.heading), dt);
    return fieldToRobot(vx, vy, std::clamp(turn, -max_turn_rate_, max_turn_rate_), pose.heading);
}

} // namespace navigatr
