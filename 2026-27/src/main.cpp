#include "main.h"

#include <cstdio>

#include "display.h"
#include "localization.h"
#include "navigation.h"

// Starts against the red wall, halfway up it, facing the audience. docs/field-frame.md.
navigatr::Route autonRoute() {
    return navigatr::Route({7.5, 69.5, 270})
        .bezier({12, 70.5}, {7, 30}, {12, 12})
        .facing(180)
        .bezier({22.5, 12}, {23, 24}, {35.5, 22.5})
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
    startAuton();
    navigator.wait();
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    pros::MotorGroup fl_mg({-1, 2}); // front left
    pros::MotorGroup fr_mg({5, -6}); // front right
    pros::MotorGroup rl_mg({3, -4}); // rear left
    pros::MotorGroup rr_mg({-7, 8}); // rear right

    while (true) {
        // A runs the auton route, B stops it. The sticks do nothing meanwhile.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) && !navigator.isRunning()) {
            startAuton();
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            navigator.cancel();
        }
        if (navigator.isRunning()) {
            pros::delay(20);
            continue;
        }

        // Mecanum: left stick drives and strafes, right stick turns.
        const int dir    = master.get_analog(ANALOG_LEFT_Y);
        const int strafe = master.get_analog(ANALOG_LEFT_X);
        const int turn   = master.get_analog(ANALOG_RIGHT_X);
        fl_mg.move(dir + strafe + turn);
        fr_mg.move(dir - strafe - turn);
        rl_mg.move(dir - strafe + turn);
        rr_mg.move(dir + strafe - turn);
        pros::delay(20);
    }
}
