// follower.h
// Stage 3, follow: pose + goal -> command, every tick. Portable.

#pragma once
#include <cmath>

#include "navigatr/config.h"
#include "navigatr/path.h"
#include "navigatr/profiles/profile.h"
#include "navigatr/types.h"

namespace navigatr
{

// Where the robot should be now, and how fast it should be moving.
struct Target {
    double x         = 0;
    double y         = 0;
    double heading   = 0; // deg
    double vx        = 0; // in/s, field frame
    double vy        = 0; // in/s
    double turn_rate = 0; // deg/s
};

// The step being followed. Each follower takes what it needs.
struct Goal {
    Target            target;               // the trajectory at t; a turn: the pose to reach
    const Path*       path       = nullptr; // a move: the whole path; a turn: null
    const Trajectory* trajectory = nullptr; // a move: its speed and time per point
    double            t          = 0;       // s since the step started
};

class Follower {
  public:
    virtual ~Follower() = default;
    virtual const char*   name() const = 0;
    virtual bool          begin(const Config&) { return true; }
    virtual void          reset() {} // a new step starts
    virtual ChassisSpeeds update(const Pose& pose, const Goal& goal, double dt) = 0;
};

// Field frame velocity (in/s) and turn (deg/s) to a robot frame command.
inline ChassisSpeeds fieldToRobot(double vx, double vy, double turn, double heading) {
    const double  h = radians(heading);
    ChassisSpeeds speeds;
    speeds.forward = vx * std::cos(h) + vy * std::sin(h);
    speeds.left    = -vx * std::sin(h) + vy * std::cos(h);
    speeds.turn    = turn;
    return speeds;
}

} // namespace navigatr
