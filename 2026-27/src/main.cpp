#include "main.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include "display.h"
#include "localization.h"
#include "navigation.h"
#include "navigatr/driver/heading_hold.h"

// Starts against the red wall, halfway up it, facing blue. docs/field-frame.md.
navigatr::Route autonRoute() {
    return navigatr::Route({7.5, 69.5, 0})
        .bezier({12, 70.5}, {5.5, 47.5}, {13, 46.5})
        .facing(0)
        .bezier({16, 16.5}, {11, 11}, {11, 12})
        .facing(180)
        .bezier({17, 15.5}, {26, 26}, {36, 23})
        .facing(0);
}

void initialize() {
    message(0, "IMU calibrating, keep still");

    const bool started = localizer.start(); // about 2 s
    std::printf("%s\n", localizer.describe().c_str());
    localizer.setPose(autonRoute().startPose()); // place the robot there before running

    startDisplay();
    if (!started) {
        message(6, "localizer: a stage failed, see terminal");
    }
}

void disabled() {}

void competition_initialize() {}

// The robot must sit at the route's start pose: the pose is reset to it.
void startAuton() {
    const navigatr::Route route = autonRoute();
    localizer.setPose(route.startPose());
    navigator.start(route);
}

void autonomous() {
    localizer.startLog("auton");
    startAuton();
    navigator.wait();
    localizer.stopLog();
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    pros::MotorGroup fl_mg({-1, 2}); // front left
    pros::MotorGroup fr_mg({5, -6}); // front right
    pros::MotorGroup rl_mg({3, -4}); // rear left
    pros::MotorGroup rr_mg({-7, 8}); // rear right

    navigatr::HeadingHold heading_hold; // kp: CHECK on the robot
    bool                  logging_route = false;
    while (true) {
        // A runs the auton route, B stops it. The sticks do nothing meanwhile.
        // Each run is logged to its own file.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) && !navigator.isRunning()) {
            logging_route = localizer.startLog("auton");
            startAuton();
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            navigator.cancel();
        }
        if (logging_route && !navigator.isRunning()) {
            localizer.stopLog();
            logging_route = false;
        }
        if (navigator.isRunning()) {
            heading_hold.reset();
            pros::delay(20);
            continue;
        }

        // Mecanum: left stick drives and strafes, right stick turns. With the
        // right stick centered, the IMU holds the heading so strafing does not twist.
        const double dir    = master.get_analog(ANALOG_LEFT_Y);
        const double strafe = master.get_analog(ANALOG_LEFT_X);
        const double turn   = heading_hold.turn(dir, strafe, master.get_analog(ANALOG_RIGHT_X),
                                                localizer.getHeading(), pros::millis());
        const std::array<double, 4> wheel = {
            dir + strafe + turn, // front left
            dir - strafe - turn, // front right
            dir - strafe + turn, // rear left
            dir + strafe - turn, // rear right
        };
        // Scaled down together past 127, so a full stick keeps its direction and the correction.
        double largest = 127;
        for (const double w : wheel) {
            largest = std::max(largest, std::fabs(w));
        }
        fl_mg.move(static_cast<int32_t>(std::lround(wheel[0] * 127 / largest)));
        fr_mg.move(static_cast<int32_t>(std::lround(wheel[1] * 127 / largest)));
        rl_mg.move(static_cast<int32_t>(std::lround(wheel[2] * 127 / largest)));
        rr_mg.move(static_cast<int32_t>(std::lround(wheel[3] * 127 / largest)));
        pros::delay(20);
    }
}
