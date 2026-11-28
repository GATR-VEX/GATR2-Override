// trajectory_follower.h
// For drives that move any way while facing any way (mecanum): tracks the
// trajectory point at time t, its velocity plus PID on x, y and heading.
// Accurate while the robot keeps up with the trajectory; once it falls
// behind, the target runs ahead and curves get cut.

#pragma once
#include "navigatr/pid.h"
#include "navigatr/followers/follower.h"

namespace navigatr
{

class TrajectoryFollower : public Follower {
  public:
    const char*   name() const override { return "trajectory"; }
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
