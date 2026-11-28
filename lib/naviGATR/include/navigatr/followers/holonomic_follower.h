// holonomic_follower.h
// For drives that move any way while facing any way (mecanum): the
// trajectory's velocity at t plus PID on x, y and heading, in the robot frame.

#pragma once
#include "navigatr/pid.h"
#include "navigatr/followers/follower.h"

namespace navigatr
{

class HolonomicFollower : public Follower {
  public:
    const char*   name() const override { return "holonomic"; }
    bool          begin(const Config& config) override;
    void          reset() override;
    ChassisSpeeds update(const Pose& pose, const Goal& goal, double dt) override;

  private:
    Pid    x_;
    Pid    y_;
    Pid    heading_;
    double max_turn_rate_ = 0;
};

} // namespace navigatr
