// sticks_gtest.cpp

#include "navigatr/driver/sticks.h"

#include <gtest/gtest.h>

#include "navigatr/drives/mecanum_drive.h"

namespace navigatr
{
namespace
{

gatr2::DriveMotors motors() {
    gatr2::DriveMotors m;
    m.motor_rpm      = 200;
    m.gear_ratio     = 1.0;
    m.wheel_diameter = 4.0;
    m.track_width    = 12.6;
    m.wheelbase      = 14.25;
    return m;
}

TEST(Sticks, Deadband) {
    EXPECT_DOUBLE_EQ(deadband(4), 0);
    EXPECT_DOUBLE_EQ(deadband(-4), 0);
    EXPECT_DOUBLE_EQ(deadband(5), 5);
    EXPECT_DOUBLE_EQ(deadband(-127), -127);
}

TEST(Sticks, MecanumTop) {
    const ChassisSpeeds top = mecanumTop(motors());
    EXPECT_NEAR(top.forward, 41.888, 0.001); // 200 rpm, 4 in wheel
    EXPECT_DOUBLE_EQ(top.left, top.forward);
    EXPECT_NEAR(top.turn, degrees(41.888 / 13.425), 0.01);
}

TEST(Sticks, FullStickIsTopSpeedInTheRobotFrame) {
    const ChassisSpeeds top = {40, 30, 180};
    const ChassisSpeeds f   = stickSpeeds({127, 0, 0}, top);
    EXPECT_DOUBLE_EQ(f.forward, 40);
    const ChassisSpeeds s = stickSpeeds({0, 127, 0}, top);
    EXPECT_DOUBLE_EQ(s.left, -30); // stick right, robot right
    const ChassisSpeeds t = stickSpeeds({0, 0, 127}, top);
    EXPECT_DOUBLE_EQ(t.turn, -180); // stick right, clockwise
    const ChassisSpeeds half = stickSpeeds({-63.5, 0, 0}, top);
    EXPECT_DOUBLE_EQ(half.forward, -20);
}

TEST(Sticks, FullStickIsFullVoltage) {
    const gatr2::DriveMotors m = motors();
    Feedforward              ff;
    ff.kv = wheelKv(m, ff);
    for (const Sticks& sticks : {Sticks{127, 0, 0}, Sticks{0, 127, 0}, Sticks{0, 0, -127}}) {
        const std::array<double, 4> mv = mecanumVoltages(stickSpeeds(sticks, mecanumTop(m)), m, ff);
        for (const double v : mv) {
            EXPECT_NEAR(std::fabs(v), ff.max_mv, 1e-6);
        }
    }
}

TEST(Sticks, StickRightGoesTheSameWayAsTheOldMixing) {
    // Old driver control: front left = dir + strafe + turn, front right = dir - strafe - turn.
    const gatr2::DriveMotors m = motors();
    Feedforward              ff;
    ff.kv = wheelKv(m, ff);
    const std::array<double, 4> strafe = mecanumVoltages(stickSpeeds({0, 60, 0}, mecanumTop(m)), m, ff);
    EXPECT_GT(strafe[0], 0); // front left
    EXPECT_LT(strafe[1], 0); // front right
    EXPECT_LT(strafe[2], 0); // back left
    EXPECT_GT(strafe[3], 0); // back right
    const std::array<double, 4> turn = mecanumVoltages(stickSpeeds({0, 0, 60}, mecanumTop(m)), m, ff);
    EXPECT_GT(turn[0], 0);
    EXPECT_LT(turn[1], 0);
    EXPECT_GT(turn[2], 0);
    EXPECT_LT(turn[3], 0);
}

} // namespace
} // namespace navigatr
