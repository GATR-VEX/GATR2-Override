// display.h
// The pose on the Brain screen and in spectaGATR, in src/display.cpp.

#pragma once
#include "spectagatr/viewer.h"

extern spectagatr::Viewer viz;

// From initialize(), after localizer.start(). Starts the viewer and a task
// that shows the pose and the localizer's status every 50 ms.
void startDisplay();
