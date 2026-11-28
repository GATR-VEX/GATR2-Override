// mecanum_drive.cpp
// The portable part: wheel voltages. The motors are in pros/.

#include "navigatr/drives/mecanum_drive.h"

#include <algorithm>
#include <cmath>

namespace navigatr
{

namespace
{

constexpr double kStillInPerS = 0.1; // below this a wheel gets no ks

} // namespace

double wheelKv(const gatr2::DriveMotors& motors, const Feedforward& feedforward) {
    if (feedforward.kv > 0) {
        return feedforward.kv;
    }
    const double top = topSpeed(motors);
    return top > 0 ? feedforward.max_mv / top : 0.0;
}

ChassisSpeeds mecanumTop(const gatr2::DriveMotors& motors) {
    const double  k = (motors.track_width + motors.wheelbase) / 2.0;
    ChassisSpeeds top;
    top.forward = topSpeed(motors);
    top.left    = top.forward;
    top.turn    = k > 0 ? degrees(top.forward / k) : 0.0;
    return top;
}

// Strafe left: front left and back right backward. Turn left: left side
// backward.
std::array<double, 4> mecanumVoltages(const ChassisSpeeds& speeds, const gatr2::DriveMotors& motors,
                                      const Feedforward& feedforward) {
    const double k    = (motors.track_width + motors.wheelbase) / 2.0;
    const double turn = radians(speeds.turn) * k;
    const std::array<double, 4> wheel = {
        speeds.forward - speeds.left - turn, // front left
        speeds.forward + speeds.left + turn, // front right
        speeds.forward + speeds.left - turn, // back left
        speeds.forward - speeds.left + turn, // back right
    };

    std::array<double, 4> mv{};
    double                largest = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        const double static_mv =
            std::fabs(wheel[i]) > kStillInPerS ? std::copysign(feedforward.ks, wheel[i]) : 0.0;
        mv[i]   = feedforward.kv * wheel[i] + static_mv;
        largest = std::max(largest, std::fabs(mv[i]));
    }
    if (largest > feedforward.max_mv) {
        for (double& m : mv) {
            m *= feedforward.max_mv / largest;
        }
    }
    return mv;
}

} // namespace navigatr
