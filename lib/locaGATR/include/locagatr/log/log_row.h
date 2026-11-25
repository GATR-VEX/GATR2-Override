// log_row.h
// One CSV row per tick: the raw readings, the published pose, and a pose set
// just before the tick. Columns: logHeader(). Empty cell: no reading. Portable.

#pragma once
#include <string>

#include "locagatr/sources/source.h"
#include "locagatr/types.h"

namespace locagatr
{

struct LogRow {
    SensorMap sensors;
    Pose      pose;             // published this tick
    bool      pose_set = false; // setPose() took effect before this tick
    Pose      set;              // its x, y and heading
};

// Column names and the line end.
std::string logHeader();

// Appends one line.
void appendLogRow(const LogRow& row, std::string& out);

} // namespace locagatr
