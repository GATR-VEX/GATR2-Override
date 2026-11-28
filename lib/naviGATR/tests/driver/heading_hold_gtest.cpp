// heading_hold_gtest.cpp

#include "navigatr/driver/heading_hold.h"

#include <gtest/gtest.h>

namespace navigatr
{
namespace
{

HeadingHoldConfig config() {
    HeadingHoldConfig c;
    c.kp        = 2.0;
    c.max_turn  = 50;
    c.deadband  = 5;
    c.settle_ms = 250;
    return c;
}

TEST(HeadingHold, StrafingHoldsTheHeadingItStartedAt) {
    HeadingHold hold(config());
    EXPECT_DOUBLE_EQ(hold.turn(0, 100, 0, 90, 0), 0); // starts holding 90
    EXPECT_TRUE(hold.holding());
    EXPECT_DOUBLE_EQ(hold.target(), 90);
    EXPECT_DOUBLE_EQ(hold.turn(0, 100, 0, 95, 10), 10);  // twisted left: turn right
    EXPECT_DOUBLE_EQ(hold.turn(0, 100, 0, 85, 20), -10); // twisted right: turn left
}

TEST(HeadingHold, CorrectionIsClamped) {
    HeadingHold hold(config());
    hold.turn(100, 0, 0, 0, 0);
    EXPECT_DOUBLE_EQ(hold.turn(100, 0, 0, 90, 10), 50);
    EXPECT_DOUBLE_EQ(hold.turn(100, 0, 0, -90, 20), -50);
}

TEST(HeadingHold, WrapsAcrossZero) {
    HeadingHold hold(config());
    hold.turn(0, 100, 0, 359, 0);
    EXPECT_DOUBLE_EQ(hold.turn(0, 100, 0, 1, 10), 4); // 2 deg counterclockwise
}

TEST(HeadingHold, TurnStickWinsThenHoldsWhereTheRobotStops) {
    HeadingHold hold(config());
    hold.turn(0, 100, 0, 0, 0);
    EXPECT_DOUBLE_EQ(hold.turn(0, 100, 60, 10, 10), 60);
    EXPECT_FALSE(hold.holding());

    // Let go: nothing while it settles.
    EXPECT_DOUBLE_EQ(hold.turn(0, 100, 0, 30, 100), 0);
    EXPECT_DOUBLE_EQ(hold.turn(0, 100, 0, 40, 200), 0);
    EXPECT_FALSE(hold.holding());

    // Then holds where it came to rest, not where it was before turning.
    EXPECT_DOUBLE_EQ(hold.turn(0, 100, 0, 45, 300), 0);
    EXPECT_DOUBLE_EQ(hold.target(), 45);
    EXPECT_DOUBLE_EQ(hold.turn(0, 100, 0, 47, 310), 4);
}

TEST(HeadingHold, StandingStillHoldsNothing) {
    HeadingHold hold(config());
    hold.turn(0, 100, 0, 0, 0);
    EXPECT_DOUBLE_EQ(hold.turn(0, 0, 0, 20, 10), 0); // pushed while still: not fought
    EXPECT_FALSE(hold.holding());
    hold.turn(100, 0, 0, 20, 20);
    EXPECT_DOUBLE_EQ(hold.target(), 20); // moving again: holds the new heading
}

TEST(HeadingHold, SmallStickNoiseIsCentered) {
    HeadingHold hold(config());
    hold.turn(0, 100, 0, 0, 0);
    EXPECT_DOUBLE_EQ(hold.turn(0, 100, 4, 5, 10), 10); // turn 4 is inside the deadband
    EXPECT_TRUE(hold.holding());
}

} // namespace
} // namespace navigatr
