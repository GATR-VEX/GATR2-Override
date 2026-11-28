// pure_pursuit_follower.h
// For drives that move any way while facing any way (mecanum): drives toward
// the point lookahead inches further along the path than the path point
// nearest the robot. Speed comes from the path's speed limits where the
// robot is, not from the clock, so a robot that falls behind does not chase
// a target running away from it. Heading follows the path. Turn steps, and
// the last lookahead inches of a move, are TrajectoryFollower's PID on the
// end pose. Settings: Config::pure_pursuit.

#pragma once
#include <cstddef>
#include <vector>

#include "navigatr/followers/follower.h"
#include "navigatr/followers/trajectory_follower.h"

namespace navigatr
{

class PurePursuitFollower : public Follower {
  public:
    const char*   name() const override { return "pure pursuit"; }
    bool          begin(const Config& config) override;
    void          reset() override;
    ChassisSpeeds update(const Pose& pose, const Goal& goal, double dt) override;

    // Last update: the nearest path point, the point it drove toward, and
    // whether it was on the last stretch.
    std::size_t nearest() const { return nearest_; }
    Point       lookahead() const { return lookahead_; }
    bool        finishing() const { return finishing_; }

  private:
    PurePursuit         config_;
    Limits              limits_;
    TrajectoryFollower  hold_; // turns and the last stretch
    Pid                 heading_;
    const Path*         path_ = nullptr; // the speeds below are for it
    std::vector<double> speeds_;         // in/s per path point
    std::size_t         nearest_   = 0;
    double              speed_     = 0; // in/s, last command
    Point               lookahead_;
    bool                finishing_ = false;
};

} // namespace navigatr
