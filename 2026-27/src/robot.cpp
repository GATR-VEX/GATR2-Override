// robot.cpp
// Ports and geometry. Inches.
// CHECK: not yet measured on this robot.

#include "robot.h"

gatr2::Robot robot() {
    gatr2::Robot r;
    r.imu_port   = 15; // CHECK
    r.board_port = 16; // CHECK

    gatr2::TrackingWheel left;
    left.name           = "left";
    left.port           = 0;    // TW0
    left.axis           = gatr2::Axis::kForward;
    left.counts_per_rev = 4000; // AS5047P 1000 PPR, x4. CHECK: spin 10 turns
    left.diameter       = 1.89; // 48 mm
    left.direction      = -1;   // counts down pushing forward
    left.offset         = 6.73; // left of center: 18.7 - 1.6 = 17.1 cm

    gatr2::TrackingWheel back;
    back.name           = "back";
    back.port           = 1;    // TW1
    back.axis           = gatr2::Axis::kSideways;
    back.counts_per_rev = 4000;
    back.diameter       = 1.89;
    back.direction      = 1;    // CHECK: counts up pushing left
    back.offset         = -3.94; // behind center: 18.4 - 8.4 = 10.0 cm

    r.tracking_wheels = {left, back};

    // Same ports as opcontrol().
    r.drive.front_left     = {-1, 2};
    r.drive.front_right    = {5, -6};
    r.drive.back_left      = {3, -4};
    r.drive.back_right     = {-7, 8};
    r.drive.motor_rpm      = 200; // CHECK: cartridge
    r.drive.gear_ratio     = 1.0; // CHECK
    r.drive.wheel_diameter = 4.0; // CHECK
    r.drive.mecanum        = true;
    r.drive.track_width    = 12.6;  // 32 cm, wheel center to wheel center
    r.drive.wheelbase      = 14.25; // 36.2 cm
    return r;
}
