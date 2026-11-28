#include "main.h"

#include <cstdio>

#include "display.h"
#include "localization.h"
#include "navigation.h"
#include "navigatr/driver/heading_hold.h"
#include "navigatr/driver/sticks.h"

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
    if (!startDrive()) {
        message(7, "drive: a setting in robot.cpp is missing");
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

        // Left stick drives and strafes, right stick turns. With the right
        // stick centered, the IMU holds the heading so strafing does not twist.
        const double dir        = navigatr::deadband(master.get_analog(ANALOG_LEFT_Y));
        const double strafe     = navigatr::deadband(master.get_analog(ANALOG_LEFT_X));
        const double stick_turn = navigatr::deadband(master.get_analog(ANALOG_RIGHT_X));
        const double turn =
            heading_hold.turn(dir, strafe, stick_turn, localizer.getHeading(), pros::millis());
        drive.apply(navigatr::stickSpeeds({dir, strafe, turn}, drive.top()));
        pros::delay(20);
    }
}
