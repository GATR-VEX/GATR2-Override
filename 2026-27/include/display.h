// display.h
// The pose on the Brain screen and in spectaGATR, in src/display.cpp.

#pragma once
#include "spectagatr/viewer.h"

extern spectagatr::Viewer viz;

// From initialize(), after localizer.start(). Starts the viewer and a task
// that shows the pose and the localizer's status every 50 ms, on the Brain
// screen and on the controller.
void startDisplay();

// One line of text on the Brain screen; lines 0 to 4 are the status.
void message(int line, const char* text);
