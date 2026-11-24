#include "main.h"

#include <cstdio>

#include "display.h"
#include "localization.h"
#include "navigation.h"

// Bench start: robot center over the tile seam crossing one tile from the red
// wall and one from the audience wall, facing blue. docs/field-frame.md.
constexpr locagatr::Pose kStart{23.4, 23.4, 0};

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "IMU calibrating, keep still");

    const bool started = localizer.start(); // about 2 s
    std::printf("%s\n", localizer.describe().c_str());
    localizer.setPose(kStart);

    startDisplay();
    if (!started) {
        pros::lcd::set_text(6, "localizer: a stage failed, see terminal");
    }
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // First test: 24 in forward, then face left.
    const navigatr::Route route = navigatr::Route(kStart).line({47.4, 23.4}).turnTo(90);
    localizer.setPose(route.startPose());
    navigator.follow(route);
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    pros::MotorGroup fl_mg({-1, 2}); // front left
    pros::MotorGroup fr_mg({5, -6}); // front right
    pros::MotorGroup rl_mg({3, -4}); // rear left
    pros::MotorGroup rr_mg({-7, 8}); // rear right

    while (true) {
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
