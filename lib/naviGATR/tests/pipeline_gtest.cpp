// pipeline_gtest.cpp
// Whole routes against a simulated robot that moves exactly as commanded, in
// simulated time.

#include "navigatr/pipeline.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "navigatr/followers/pure_pursuit_follower.h"
#include "navigatr/followers/trajectory_follower.h"

namespace navigatr
{
namespace
{

struct SimRobot : Drive {
    Pose          pose;
    ChassisSpeeds speeds;
    uint32_t      now_ms  = 0;
    bool          stuck   = false; // ignores every command
    double        scale   = 1;     // moves this much of what it is told, like a slow strafe
    int           applies = 0;
    std::vector<Pose> trail;       // every ms

    const char* name() const override { return "sim"; }
    void        apply(const ChassisSpeeds& s) override {
        speeds = stuck ? ChassisSpeeds{} : s;
        ++applies;
    }
    void stop() override { speeds = ChassisSpeeds{}; }

    // 1 ms steps in the field frame.
    void sleepUntil(uint32_t ms) {
        for (; now_ms < ms; ++now_ms) {
            const double h = radians(pose.heading);
            const double dt = 0.001 * scale;
            pose.x += (speeds.forward * std::cos(h) - speeds.left * std::sin(h)) * dt;
            pose.y += (speeds.forward * std::sin(h) + speeds.left * std::cos(h)) * dt;
            pose.heading = std::fmod(pose.heading + speeds.turn * dt + 360.0, 360.0);
            trail.push_back(pose);
        }
    }

    Io io() {
        Io io;
        io.pose        = [this] { return pose; };
        io.now_ms      = [this] { return now_ms; };
        io.sleep_until = [this](uint32_t ms) { sleepUntil(ms); };
        return io;
    }
};

Config config(SimRobot& robot) {
    Config c;
    c.drive = &robot;
    return c;
}

TEST(Pipeline, NeedsADrive) {
    SimRobot robot;
    Pipeline pipeline(Config{}, robot.io());
    EXPECT_FALSE(pipeline.begin());
}

TEST(Pipeline, FollowsLinesCurvesAndTurns) {
    SimRobot robot;
    robot.pose        = {24, 12, 90};
    const Route route = Route({24, 12, 90})
                            .line({24, 36})
                            .bezier({24, 48}, {36, 60}, {48, 60})
                            .facing(0) // a mecanum robot turns while it drives
                            .turnTo(180)
                            .wait(200)
                            .line({24, 60});
    Pipeline pipeline(config(robot), robot.io());
    ASSERT_TRUE(pipeline.begin());
    EXPECT_TRUE(pipeline.run(route).ok());
    EXPECT_NEAR(robot.pose.x, 24, 1.0);
    EXPECT_NEAR(robot.pose.y, 60, 1.0);
    EXPECT_NEAR(wrap180(robot.pose.heading - 180), 0, 2.0);
    EXPECT_DOUBLE_EQ(robot.speeds.forward, 0); // stopped at the end
}

TEST(Pipeline, CorrectsAStartingError) {
    SimRobot robot;
    robot.pose = {26, 10, 80}; // off by 2 in, 2 in, 10 deg
    Pipeline pipeline(config(robot), robot.io());
    ASSERT_TRUE(pipeline.begin());
    EXPECT_TRUE(pipeline.run(Route({24, 12, 90}).line({24, 48})).ok());
    EXPECT_NEAR(robot.pose.x, 24, 1.0);
    EXPECT_NEAR(robot.pose.y, 48, 1.0);
}

TEST(Pipeline, ActionsRunOnceInOrder) {
    SimRobot                 robot;
    std::vector<std::string> log;
    const Route              route = Route({0, 0, 0})
                            .line({24, 0})
                            .at(0.5, [&] { log.push_back("half"); })
                            .at(1.0, [&] { log.push_back("end"); })
                            .wait(50)
                            .at(0, [&] { log.push_back("wait"); });
    Pipeline pipeline(config(robot), robot.io());
    ASSERT_TRUE(pipeline.begin());
    pipeline.run(route);
    EXPECT_EQ(log, (std::vector<std::string>{"half", "end", "wait"}));
}

TEST(Pipeline, StuckRobotTimesOutAndMovesOn) {
    SimRobot robot;
    robot.stuck = true;
    Config c    = config(robot);
    c.tolerances.timeout_ms = 500;
    Pipeline pipeline(c, robot.io());
    ASSERT_TRUE(pipeline.begin());
    int         after = 0;
    const Route route = Route({0, 0, 0}).line({24, 0}).wait(10).at(0, [&] { ++after; });
    const Outcome outcome = pipeline.run(route);
    EXPECT_FALSE(outcome.ok());
    EXPECT_EQ(outcome.timed_out, 1u);
    EXPECT_EQ(outcome.first_timed_out, 0);
    EXPECT_FALSE(outcome.cancelled);
    EXPECT_EQ(after, 1); // the rest of the route still ran
    EXPECT_LT(robot.now_ms, 5000u);
}

TEST(Pipeline, PlansTheWholeRouteBeforeMoving) {
    SimRobot robot;
    Pipeline pipeline(config(robot), robot.io());
    ASSERT_TRUE(pipeline.begin());
    const Plan plan =
        pipeline.plan(Route({0, 0, 0}).line({24, 0}).turnTo(90).line({24, 36}).wait(100));
    ASSERT_EQ(plan.moves.size(), 2u);
    EXPECT_NEAR(plan.length, 60, 1e-6);
    EXPECT_NEAR(plan.duration, plan.moves[0].trajectory.duration() + plan.moves[1].trajectory.duration(),
                1e-9);
    EXPECT_EQ(robot.applies, 0);
}

TEST(Pipeline, ProgressIsDrivingDistanceCovered) {
    SimRobot            robot;
    std::vector<double> seen;
    Io                  io = robot.io();
    io.progress            = [&](double f) { seen.push_back(f); };
    double at_turn         = -1;
    Pipeline pipeline(config(robot), io);
    ASSERT_TRUE(pipeline.begin());
    const Route route =
        Route({0, 0, 0}).line({24, 0}).turnTo(90).at(0, [&] { at_turn = seen.back(); }).line({24, 72});
    EXPECT_TRUE(pipeline.run(route).ok());
    ASSERT_FALSE(seen.empty());
    EXPECT_DOUBLE_EQ(seen.front(), 0);
    EXPECT_DOUBLE_EQ(seen.back(), 1);
    for (std::size_t i = 1; i < seen.size(); ++i) {
        EXPECT_GE(seen[i], seen[i - 1]);
    }
    EXPECT_NEAR(at_turn, 0.25, 1e-6); // 24 of 96 in
}

TEST(Pipeline, CancelStopsTheRouteAndTheDrive) {
    SimRobot robot;
    Io       io  = robot.io();
    io.cancelled = [&] { return robot.pose.x > 12; };
    int after    = 0;
    Pipeline pipeline(config(robot), io);
    ASSERT_TRUE(pipeline.begin());
    const Route   route   = Route({0, 0, 0}).line({48, 0}).at(1.0, [&] { ++after; }).wait(10);
    const Outcome outcome = pipeline.run(route);
    EXPECT_TRUE(outcome.cancelled);
    EXPECT_FALSE(outcome.ok());
    EXPECT_EQ(after, 0); // nothing after the cancel ran
    EXPECT_LT(robot.pose.x, 14);
    EXPECT_DOUBLE_EQ(robot.speeds.forward, 0);
}

TEST(Pipeline, StepsRunOneAtATimeWithWorkBetween) {
    SimRobot robot;
    Pipeline pipeline(config(robot), robot.io());
    ASSERT_TRUE(pipeline.begin());
    const Route route = Route({0, 0, 0}).line({24, 0}).turnTo(90).line({24, 36});
    const Plan  plan  = pipeline.plan(route);

    std::vector<Pose> between;
    for (std::size_t i = 0; i < route.steps().size(); ++i) {
        EXPECT_TRUE(pipeline.runStep(route, plan, i).ok());
        EXPECT_DOUBLE_EQ(robot.speeds.forward, 0); // stopped after each step
        between.push_back(robot.pose);
        robot.sleepUntil(robot.now_ms + 500); // scoring, nothing moving
    }
    ASSERT_EQ(between.size(), 3u);
    EXPECT_NEAR(between[0].x, 24, 1.0);
    EXPECT_NEAR(wrap180(between[1].heading - 90), 0, 2.0);
    EXPECT_NEAR(between[2].x, 24, 1.0);
    EXPECT_NEAR(between[2].y, 36, 1.0);
}

TEST(Pipeline, StepProgressCoversThatStepOnly) {
    SimRobot            robot;
    std::vector<double> seen;
    Io                  io = robot.io();
    io.progress            = [&](double f) { seen.push_back(f); };
    double half_way        = -1;
    Pipeline pipeline(config(robot), io);
    ASSERT_TRUE(pipeline.begin());
    const Route route = Route({0, 0, 0}).line({24, 0}).turnTo(90).line({24, 72}).at(
        0.5, [&] { half_way = seen.back(); });
    const Plan plan = pipeline.plan(route);
    pipeline.runStep(route, plan, 0);
    pipeline.runStep(route, plan, 1);
    seen.clear();
    EXPECT_TRUE(pipeline.runStep(route, plan, 2).ok());
    ASSERT_FALSE(seen.empty());
    EXPECT_DOUBLE_EQ(seen.front(), 0);
    EXPECT_DOUBLE_EQ(seen.back(), 1);
    EXPECT_NEAR(half_way, 0.5, 0.05); // half of this 72 in line, not of the route
}

TEST(Pipeline, StepPastTheEndDoesNothing) {
    SimRobot robot;
    Pipeline pipeline(config(robot), robot.io());
    ASSERT_TRUE(pipeline.begin());
    const Route route = Route({0, 0, 0}).line({24, 0});
    const Plan  plan  = pipeline.plan(route);
    pipeline.runStep(route, plan, 1);
    EXPECT_EQ(robot.applies, 0);
    EXPECT_EQ(robot.now_ms, 0u);
}

TEST(Pipeline, TimedOutStepReportsItsIndex) {
    SimRobot robot;
    robot.stuck             = true;
    Config c                = config(robot);
    c.tolerances.timeout_ms = 200;
    Pipeline pipeline(c, robot.io());
    ASSERT_TRUE(pipeline.begin());
    const Route   route   = Route({0, 0, 0}).wait(10).line({24, 0});
    const Outcome outcome = pipeline.runStep(route, pipeline.plan(route), 1);
    EXPECT_EQ(outcome.timed_out, 1u);
    EXPECT_EQ(outcome.first_timed_out, 1);
}

// Farthest the robot got from the path, in.
double worstMiss(const std::vector<Pose>& trail, const Path& path) {
    double worst = 0;
    for (const Pose& pose : trail) {
        double best = 1e9;
        for (const PathPoint& p : path.points) {
            best = std::min(best, std::hypot(p.x - pose.x, p.y - pose.y));
        }
        worst = std::max(worst, best);
    }
    return worst;
}

// The tests above run the default, PurePursuitFollower.

TEST(Pipeline, TrajectoryFollowerFollowsLinesCurvesAndTurns) {
    SimRobot           robot;
    TrajectoryFollower trajectory;
    robot.pose        = {24, 12, 90};
    const Route route = Route({24, 12, 90})
                            .line({24, 36})
                            .bezier({24, 48}, {36, 60}, {48, 60})
                            .facing(0)
                            .turnTo(180)
                            .wait(200)
                            .line({24, 60});
    Config c   = config(robot);
    c.follower = &trajectory;
    Pipeline pipeline(c, robot.io());
    ASSERT_TRUE(pipeline.begin());
    EXPECT_TRUE(pipeline.run(route).ok());
    EXPECT_NEAR(robot.pose.x, 24, 1.0);
    EXPECT_NEAR(robot.pose.y, 60, 1.0);
    EXPECT_NEAR(wrap180(robot.pose.heading - 180), 0, 2.0);
}

TEST(Pipeline, TrajectoryFollowerCorrectsAStartingError) {
    SimRobot           robot;
    TrajectoryFollower trajectory;
    robot.pose = {26, 10, 80}; // off by 2 in, 2 in, 10 deg
    Config c   = config(robot);
    c.follower = &trajectory;
    Pipeline pipeline(c, robot.io());
    ASSERT_TRUE(pipeline.begin());
    EXPECT_TRUE(pipeline.run(Route({24, 12, 90}).line({24, 48})).ok());
    EXPECT_NEAR(robot.pose.x, 24, 1.0);
    EXPECT_NEAR(robot.pose.y, 48, 1.0);
}

// Farthest from the path on a straight then a curve, the robot moving
// scale of what it is told.
double missWhenScaled(double scale, Follower* follower) {
    SimRobot robot;
    robot.scale       = scale;
    robot.pose        = {24, 12, 90};
    const Route route = Route({24, 12, 90}).line({24, 36}).bezier({24, 48}, {36, 60}, {48, 60});
    Config c          = config(robot);
    c.follower        = follower;
    Pipeline pipeline(c, robot.io());
    EXPECT_TRUE(pipeline.begin());
    EXPECT_TRUE(pipeline.run(route).ok());
    EXPECT_NEAR(robot.pose.x, 48, 1.0);
    EXPECT_NEAR(robot.pose.y, 60, 1.0);
    return worstMiss(robot.trail, pipeline.plan(route).moves[0].path);
}

TEST(Pipeline, PurePursuitStaysOnThePathWhenTheRobotIsSlow) {
    // The trajectory follower cuts the curve once the robot falls behind;
    // pure pursuit misses by what its lookahead cuts, at any speed.
    PurePursuitFollower pure_pursuit;
    TrajectoryFollower  trajectory;
    const double        full = missWhenScaled(1.0, &pure_pursuit);
    const double        slow = missWhenScaled(0.75, &pure_pursuit);
    EXPECT_LT(slow, 1.0);
    EXPECT_NEAR(slow, full, 0.1);
    EXPECT_GT(missWhenScaled(0.75, &trajectory) - missWhenScaled(1.0, &trajectory), 0.5);
}

} // namespace
} // namespace navigatr
