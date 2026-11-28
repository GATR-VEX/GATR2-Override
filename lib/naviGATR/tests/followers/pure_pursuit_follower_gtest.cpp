// pure_pursuit_follower_gtest.cpp

#include "navigatr/followers/pure_pursuit_follower.h"

#include <gtest/gtest.h>

#include "navigatr/route.h"

namespace navigatr
{
namespace
{

Config config() {
    Config c;
    c.limits.max_speed     = 40;
    c.limits.max_accel     = 60;
    c.limits.max_decel     = 60;
    c.limits.max_turn_rate = 180;
    c.gains.kp             = 3;
    c.gains.heading_kp     = 4;
    return c;
}

PurePursuitFollower started(double lookahead = 12) {
    Config c                 = config();
    c.pure_pursuit.lookahead = lookahead;
    PurePursuitFollower f;
    EXPECT_TRUE(f.begin(c));
    return f;
}

// The first move of a route, sampled.
Path pathOf(const Route& route) { return samplePath(route.steps().front().segments); }

Goal along(const Path& path, double t = 0) {
    Goal goal;
    goal.path = &path;
    goal.t    = t;
    return goal;
}

TEST(PurePursuit, StartsFromRestAndSpeedsUpAtMaxAccel) {
    PurePursuitFollower f    = started();
    const Path          path = pathOf(Route({0, 0, 0}).line({96, 0}));
    Pose                pose;
    ChassisSpeeds       s = f.update(pose, along(path), 0.01);
    EXPECT_NEAR(s.forward, 0.6, 1e-9); // 60 in/s^2 for 10 ms
    EXPECT_NEAR(s.left, 0, 1e-9);
    for (int i = 0; i < 200; ++i) {
        s = f.update(pose, along(path), 0.01);
    }
    EXPECT_NEAR(s.forward, 40, 1e-9); // max speed, not more
}

TEST(PurePursuit, SteersBackOntoThePath) {
    PurePursuitFollower f    = started();
    const Path          path = pathOf(Route({0, 0, 0}).line({96, 0}));
    Pose                pose;
    pose.y = 3; // 3 in left of the path
    for (pose.x = 0; pose.x < 30; pose.x += 1) {
        f.update(pose, along(path), 0.01);
    }
    pose.x                = 30;
    const ChassisSpeeds s = f.update(pose, along(path), 0.01);
    EXPECT_GT(s.forward, 0);
    EXPECT_LT(s.left, 0); // toward the path, on the right
    EXPECT_NEAR(f.lookahead().x, 42, 0.5); // 12 in on from the nearest point
    EXPECT_NEAR(f.lookahead().y, 0, 1e-6);
}

TEST(PurePursuit, NearestPointNeverGoesBack) {
    PurePursuitFollower f    = started();
    const Path          path = pathOf(Route({0, 0, 0}).line({96, 0}));
    Pose                pose;
    for (pose.x = 0; pose.x <= 20; pose.x += 0.5) {
        f.update(pose, along(path), 0.01);
    }
    const std::size_t at_20 = f.nearest();
    pose.x                  = 5; // pushed back
    f.update(pose, along(path), 0.01);
    EXPECT_EQ(f.nearest(), at_20);
}

TEST(PurePursuit, PathThatComesBackIsNotCutShort) {
    // Out 36 in and back: at the start the far end of the path is just as close.
    PurePursuitFollower f    = started();
    const Path          path = pathOf(Route({0, 0, 0}).line({36, 0}).line({36, 2}).line({0, 2}));
    Pose                pose;
    f.update(pose, along(path), 0.01);
    EXPECT_LT(path.points[f.nearest()].s, 1.0);
    EXPECT_FALSE(f.finishing());
}

TEST(PurePursuit, LastStretchSettlesOnTheEnd) {
    PurePursuitFollower f    = started();
    const Path          path = pathOf(Route({0, 0, 0}).line({48, 0}));
    Pose                pose;
    for (pose.x = 0; pose.x < 45; pose.x += 1) {
        f.update(pose, along(path), 0.01);
    }
    EXPECT_TRUE(f.finishing()); // within 12 in of the end since x 36
    pose.x                = 45;
    const ChassisSpeeds s = f.update(pose, along(path), 0.01);
    EXPECT_NEAR(s.forward, 3 * 3, 1e-6); // kp 3, 3 in short
    pose.x = 48;
    EXPECT_NEAR(f.update(pose, along(path), 0.01).forward, 0, 1e-6);
}

TEST(PurePursuit, HeadingFollowsThePath) {
    PurePursuitFollower f    = started();
    const Path          path = pathOf(Route({0, 0, 0}).line({96, 0}).facing(90));
    Pose                pose;
    pose.x                = 24; // the path already faces about 22 deg here
    const ChassisSpeeds s = f.update(pose, along(path), 0.01);
    EXPECT_GT(s.turn, 0); // counterclockwise toward it
}

TEST(PurePursuit, TurnStepHoldsThePose) {
    PurePursuitFollower f = started();
    Goal                turn;
    turn.target.heading = 90;
    Pose pose;
    EXPECT_GT(f.update(pose, turn, 0.01).turn, 0);
}

TEST(PurePursuit, ResetStartsTheNextMoveFromRest) {
    PurePursuitFollower f    = started();
    const Path          path = pathOf(Route({0, 0, 0}).line({96, 0}));
    Pose                pose;
    for (int i = 0; i < 100; ++i) {
        f.update(pose, along(path), 0.01);
    }
    f.reset();
    EXPECT_NEAR(f.update(pose, along(path), 0.01).forward, 0.6, 1e-9);
    EXPECT_EQ(f.nearest(), 0u);
}

TEST(PurePursuit, NeedsALookahead) {
    Config c                 = config();
    c.pure_pursuit.lookahead = 0;
    PurePursuitFollower f;
    EXPECT_FALSE(f.begin(c));
}

} // namespace
} // namespace navigatr
