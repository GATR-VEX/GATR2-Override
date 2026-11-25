// log_row.cpp

#include "locagatr/log/log_row.h"

#include <cstdio>

namespace locagatr
{

namespace
{

constexpr const char* kImuColumns[] = {"imu_rotation", "imu_pitch",   "imu_roll",
                                       "imu_gyro_x",   "imu_gyro_y",  "imu_gyro_z",
                                       "imu_accel_x",  "imu_accel_y", "imu_accel_z"};

constexpr const char* kPoseColumns[] = {"x", "y", "heading", "pitch", "roll",
                                        "set_x", "set_y", "set_heading"};

// Integers through to_string: newlib nano printf has no %llu.
void integer(std::string& out, unsigned long long value) {
    out += ',';
    out += std::to_string(value);
}

void integer(std::string& out, long long value) {
    out += ',';
    out += std::to_string(value);
}

void number(std::string& out, double value) {
    char text[32];
    std::snprintf(text, sizeof text, ",%.4f", value);
    out += text;
}

void flag(std::string& out, bool value) { out += value ? ",1" : ",0"; }

} // namespace

std::string logHeader() {
    std::string out = "t_us,board_open,board_fresh,board_restarted,board_frames,board_dropped";
    for (std::size_t i = 0; i < kBoardPorts; ++i) {
        out += ",tw" + std::to_string(i);
    }
    for (std::size_t port = 1; port < kSmartPorts; ++port) {
        out += ",m" + std::to_string(port);
    }
    for (const char* name : kImuColumns) {
        out += ',';
        out += name;
    }
    for (const char* name : kPoseColumns) {
        out += ',';
        out += name;
    }
    out += '\n';
    return out;
}

void appendLogRow(const LogRow& row, std::string& out) {
    const SensorMap&   s     = row.sensors;
    const BoardStatus& board = s.board;

    out += std::to_string(static_cast<unsigned long long>(s.time_us));
    flag(out, board.port_open);
    flag(out, board.fresh);
    flag(out, board.restarted);
    integer(out, static_cast<unsigned long long>(board.frames));
    integer(out, static_cast<unsigned long long>(board.dropped));

    for (const std::optional<int32_t>& count : s.encoder_counts) {
        if (count) {
            integer(out, static_cast<long long>(*count));
        } else {
            out += ',';
        }
    }
    for (std::size_t port = 1; port < kSmartPorts; ++port) {
        if (s.motor_positions[port]) {
            number(out, *s.motor_positions[port]);
        } else {
            out += ',';
        }
    }

    if (s.imu) {
        const ImuSample& imu = *s.imu;
        for (double value : {imu.rotation, imu.pitch, imu.roll, imu.gyro.x, imu.gyro.y, imu.gyro.z,
                             imu.accel.x, imu.accel.y, imu.accel.z}) {
            number(out, value);
        }
    } else {
        out += ",,,,,,,,,";
    }

    for (double value : {row.pose.x, row.pose.y, row.pose.heading, row.pose.pitch, row.pose.roll}) {
        number(out, value);
    }
    if (row.pose_set) {
        number(out, row.set.x);
        number(out, row.set.y);
        number(out, row.set.heading);
    } else {
        out += ",,,";
    }
    out += '\n';
}

} // namespace locagatr
