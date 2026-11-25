// robot.h
// Ports and geometry of the robot, shared by locaGATR and naviGATR. Inches.

#pragma once
#include <cstdint>
#include <vector>

namespace gatr2
{

enum class Axis : uint8_t {
    kForward,  // rolls when the robot drives forward
    kSideways, // rolls when the robot strafes left
};

struct TrackingWheel {
    const char* name           = "";   // its measurement's name, e.g. "left"
    uint8_t     port           = 0;    // control board connector: 0 = TW0
    Axis        axis           = Axis::kForward;
    double      counts_per_rev = 4000; // decoded counts per wheel turn, x4: 1000 PPR -> 4000
    double      diameter       = 2.0;  // in
    int         direction      = 1;    // -1 if it counts down going forward / left
    double      offset         = 0;    // in. Forward wheels: left of center. Sideways: ahead of center
};

// Two forward tracking wheels, by name, whose difference gives yaw.
struct WheelPair {
    const char* left  = nullptr;
    const char* right = nullptr;
};

// Drive motors by corner, as PROS MotorGroup ports: negative is reversed, so
// that every motor turns positive when the robot drives forward. A tank drive
// lists its left motors under front_left and its right under front_right.
struct DriveMotors {
    std::vector<int8_t> front_left;
    std::vector<int8_t> front_right;
    std::vector<int8_t> back_left;
    std::vector<int8_t> back_right;
    double              motor_rpm      = 200;   // cartridge: 100, 200 or 600
    double              gear_ratio     = 1.0;   // wheel turns per motor output turn
    double              wheel_diameter = 4.0;   // in
    bool                mecanum        = false; // sideways travel from the four corners
    double              track_width    = 12;    // in, left wheels to right wheels
    double              wheelbase      = 12;    // in, front wheels to back wheels
};

struct Robot {
    uint8_t                    imu_port   = 0;      // VEX Inertial Sensor, 1..21
    uint8_t                    board_port = 0;      // Smart Port wired to the control board, 1..21
    int32_t                    board_baud = 921600; // must match the board
    uint32_t                   board_stale_us = 20000; // no frame from the board for this long: not fresh
    std::vector<TrackingWheel> tracking_wheels;
    WheelPair                  wheel_pair;
    DriveMotors                drive;
};

} // namespace gatr2
