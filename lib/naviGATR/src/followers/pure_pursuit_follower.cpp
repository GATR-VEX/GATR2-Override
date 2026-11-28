// pure_pursuit_follower.cpp

#include "navigatr/followers/pure_pursuit_follower.h"

#include <algorithm>
#include <cmath>

#include "navigatr/profiles/trapezoid_profile.h"

namespace navigatr
{

namespace
{

double distance2(const PathPoint& p, const Pose& pose) {
    const double dx = p.x - pose.x;
    const double dy = p.y - pose.y;
    return dx * dx + dy * dy;
}

// The point s inches along the path, searching on from index from.
Point pointAt(const std::vector<PathPoint>& p, std::size_t from, double s) {
    for (std::size_t i = from; i + 1 < p.size(); ++i) {
        if (p[i + 1].s >= s) {
            const double ds = p[i + 1].s - p[i].s;
            const double f  = ds > 1e-9 ? (s - p[i].s) / ds : 0.0;
            return {p[i].x + f * (p[i + 1].x - p[i].x), p[i].y + f * (p[i + 1].y - p[i].y)};
        }
    }
    return {p.back().x, p.back().y};
}

} // namespace

bool PurePursuitFollower::begin(const Config& config) {
    limits_  = config.limits;
    heading_ = Pid(config.gains.heading_kp, config.gains.heading_ki, config.gains.heading_kd);
    return hold_.begin(config) && config_.lookahead > 0 && config_.search > 0 &&
           limits_.max_accel > 0;
}

void PurePursuitFollower::reset() {
    hold_.reset();
    heading_.reset();
    path_      = nullptr;
    nearest_   = 0;
    speed_     = 0;
    finishing_ = false;
}

ChassisSpeeds PurePursuitFollower::update(const Pose& pose, const Goal& goal, double dt) {
    if (goal.path == nullptr || goal.path->points.size() < 2) {
        return hold_.update(pose, goal, dt); // a turn
    }
    const Path&                   path = *goal.path;
    const std::vector<PathPoint>& p    = path.points;
    if (path_ != goal.path) {
        path_    = goal.path;
        speeds_  = speedLimits(path, limits_);
        nearest_ = 0;
        speed_   = 0;
    }

    // Nearest point, never behind the last one, so a path that comes back
    // near itself is not cut short.
    const double search_end = p[nearest_].s + config_.search;
    double       best       = distance2(p[nearest_], pose);
    for (std::size_t i = nearest_ + 1; i < p.size() && p[i].s <= search_end; ++i) {
        const double d = distance2(p[i], pose);
        if (d < best) {
            best     = d;
            nearest_ = i;
        }
    }
    const PathPoint& here = p[nearest_];

    // Last stretch: settle on the end pose.
    if (path.length() - here.s <= config_.lookahead) {
        finishing_ = true;
        lookahead_ = {p.back().x, p.back().y};
        Goal end;
        end.target.x       = p.back().x;
        end.target.y       = p.back().y;
        end.target.heading = p.back().heading;
        speed_             = 0;
        return hold_.update(pose, end, dt);
    }
    finishing_ = false;

    // Toward the lookahead point at the speed allowed here.
    lookahead_      = pointAt(p, nearest_, here.s + config_.lookahead);
    speed_          = std::min(speeds_[nearest_], speed_ + limits_.max_accel * dt);
    const double dx = lookahead_.x - pose.x;
    const double dy = lookahead_.y - pose.y;
    const double d  = std::hypot(dx, dy);
    const double ux = d > 1e-6 ? dx / d : std::cos(radians(here.tangent));
    const double uy = d > 1e-6 ? dy / d : std::sin(radians(here.tangent));

    // Heading: the path's, plus how fast it changes at this speed.
    const PathPoint& next = p[std::min(nearest_ + 1, p.size() - 1)];
    const double     ds   = next.s - here.s;
    const double     rate = ds > 1e-9 ? (next.heading - here.heading) / ds * speed_ : 0.0;
    const double turn = rate + heading_.update(wrap180(here.heading - pose.heading), dt);
    return fieldToRobot(speed_ * ux, speed_ * uy,
                        std::clamp(turn, -limits_.max_turn_rate, limits_.max_turn_rate),
                        pose.heading);
}

} // namespace navigatr
