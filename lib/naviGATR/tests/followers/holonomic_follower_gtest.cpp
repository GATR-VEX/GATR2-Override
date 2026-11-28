// holonomic_follower_gtest.cpp

#include "navigatr/followers/holonomic_follower.h"

#include <gtest/gtest.h>

namespace navigatr
{
namespace
{

HolonomicFollower started() {
    HolonomicFollower f;
    Config            c;
    c.gains.kp         = 2;
    c.gains.heading_kp = 3;
    c.limits.max_turn_rate = 90;
    EXPECT_TRUE(f.begin(c));
    return f;
}

Goal at(const Target& target) {
    Goal goal;
    goal.target = target;
    return goal;
}

TEST(HolonomicFollower, OnTargetPassesTheVelocityIntoTheRobotFrame) {
    HolonomicFollower f = started();
    Pose              pose;
    pose.heading = 90; // facing +y
    Target target;
    target.heading = 90;
    target.vx      = 0;
    target.vy      = 20; // moving +y: straight ahead
    const ChassisSpeeds s = f.update(pose, at(target), 0.01);
    EXPECT_NEAR(s.forward, 20, 1e-9);
    EXPECT_NEAR(s.left, 0, 1e-9);
    EXPECT_NEAR(s.turn, 0, 1e-9);
}

TEST(HolonomicFollower, PositionErrorPullsTowardTheTarget) {
    HolonomicFollower f = started();
    Pose              pose; // at the origin facing +x
    Target            target;
    target.y              = 5; // 5 in to the left
    const ChassisSpeeds s = f.update(pose, at(target), 0.01);
    EXPECT_NEAR(s.forward, 0, 1e-9);
    EXPECT_NEAR(s.left, 10, 1e-9); // kp 2
}

TEST(HolonomicFollower, HeadingErrorTheShortWayAndClamped) {
    HolonomicFollower f = started();
    Pose              pose;
    pose.heading = 350;
    Target target;
    target.heading = 10; // 20 deg counterclockwise
    EXPECT_NEAR(f.update(pose, at(target), 0.01).turn, 60, 1e-9);
    target.heading = 170; // -180: clamped to the limit
    EXPECT_NEAR(std::fabs(f.update(pose, at(target), 0.01).turn), 90, 1e-9);
}

} // namespace
} // namespace navigatr
