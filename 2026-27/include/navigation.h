// navigation.h
// The drive and the navigator, configured in src/navigation.cpp.

#pragma once
#include "navigatr/drives/mecanum_drive.h"
#include "navigatr/navigator.h"
#include "navigatr/route.h"

// Auton and driver control both drive through it.
extern navigatr::MecanumDrive drive;
extern navigatr::Navigator    navigator;

// From initialize(), before driver control uses drive. False: a drive
// setting in robot.cpp is missing.
bool startDrive();
