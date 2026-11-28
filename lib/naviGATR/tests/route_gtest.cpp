// route_gtest.cpp

#include "navigatr/route.h"

#include <gtest/gtest.h>

namespace navigatr
{
namespace
{

TEST(Route, SegmentsInARowAreOneMove) {
    const Route route = Route({0, 0, 90}).line({0, 24}).bezier({0, 36}, {12, 48}, {24, 48});
    ASSERT_EQ(route.steps().size(), 1u);
    const Step& move = route.steps()[0];
    EXPECT_EQ(move.kind, Step::Kind::kMove);
    ASSERT_EQ(move.segments.size(), 2u);
    EXPECT_DOUBLE_EQ(move.segments[1].curve.p0.y, 24); // starts where the line ended
    EXPECT_DOUBLE_EQ(move.segments[1].start_heading, 90);
    EXPECT_DOUBLE_EQ(move.segments[1].end_heading, 90); // heading holds by default
}

TEST(Route, PathStartsANewMove) {
    const Route route = Route({0, 0, 0}).path().line({24, 0}).facing(90).path().line({24, 24}).line({0, 24});
    ASSERT_EQ(route.steps().size(), 2u); // a leading path() adds nothing
    EXPECT_EQ(route.steps()[0].segments.size(), 1u);
    ASSERT_EQ(route.steps()[1].segments.size(), 2u);
    EXPECT_DOUBLE_EQ(route.steps()[1].segments[0].curve.p0.x, 24); // starts where the last ended
    EXPECT_DOUBLE_EQ(route.steps()[1].segments[0].start_heading, 90);
}

TEST(Route, FollowPicksTheFollowerPerMove) {
    const Route route = Route({0, 0, 0})
                            .follow(Follow::kTrajectory) // before any segment: the first move
                            .line({24, 0})
                            .line({24, 24})
                            .path()
                            .line({0, 24})
                            .follow(Follow::kPurePursuit) // after a segment: that move
                            .turnTo(90)
                            .follow(Follow::kTrajectory) // after a turn: the next move
                            .line({0, 0})
                            .path()
                            .line({12, 0}); // not given: default
    ASSERT_EQ(route.steps().size(), 5u);
    EXPECT_EQ(route.steps()[0].follow, Follow::kTrajectory);
    EXPECT_EQ(route.steps()[0].segments.size(), 2u); // one move, one follower
    EXPECT_EQ(route.steps()[1].follow, Follow::kPurePursuit);
    EXPECT_EQ(route.steps()[3].follow, Follow::kTrajectory);
    EXPECT_EQ(route.steps()[4].follow, Follow::kDefault);
}

TEST(Route, TurnAndWaitEndAMove) {
    const Route route = Route({0, 0, 0}).line({24, 0}).turnTo(90).wait(250).line({24, 24});
    ASSERT_EQ(route.steps().size(), 4u);
    EXPECT_EQ(route.steps()[1].kind, Step::Kind::kTurn);
    EXPECT_DOUBLE_EQ(route.steps()[1].position.x, 24);
    EXPECT_DOUBLE_EQ(route.steps()[1].heading, 90);
    EXPECT_EQ(route.steps()[2].ms, 250u);
    EXPECT_DOUBLE_EQ(route.steps()[3].segments[0].start_heading, 90);
}

TEST(Route, FacingAndTangentSetTheLastSegment) {
    const Route route = Route({0, 0, 0}).line({24, 0}).facing(45).line({24, 24}).tangent();
    const Step& move  = route.steps()[0];
    EXPECT_DOUBLE_EQ(move.segments[0].end_heading, 45);
    EXPECT_DOUBLE_EQ(move.segments[1].start_heading, 45);
    EXPECT_TRUE(move.segments[1].tangent);
    EXPECT_NEAR(move.segments[1].end_heading, 90, 1e-9);
    EXPECT_NEAR(route.endPose().heading, 90, 1e-9);
    EXPECT_DOUBLE_EQ(route.endPose().y, 24);
}

TEST(Route, ActionsGoOnTheLastSegmentOrStep) {
    int         a = 0, b = 0;
    const Route route =
        Route({0, 0, 0}).line({24, 0}).at(0.5, [&] { ++a; }).wait(100).at(0, [&] { ++b; });
    ASSERT_EQ(route.steps()[0].segments[0].actions.size(), 1u);
    EXPECT_DOUBLE_EQ(route.steps()[0].segments[0].actions[0].at, 0.5);
    ASSERT_EQ(route.steps()[1].actions.size(), 1u);
    route.steps()[1].actions[0].run();
    EXPECT_EQ(b, 1);
}

} // namespace
} // namespace navigatr
