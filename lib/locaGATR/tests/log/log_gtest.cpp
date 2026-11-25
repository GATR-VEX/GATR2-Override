// log_gtest.cpp

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <sstream>
#include <thread>
#include <vector>

#include "locagatr/log/log_queue.h"
#include "locagatr/log/log_row.h"

namespace locagatr
{
namespace
{

std::vector<std::string> cells(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream        in(line.substr(0, line.find('\n')));
    std::string              cell;
    while (std::getline(in, cell, ',')) {
        out.push_back(cell);
    }
    if (!line.empty() && line[line.find('\n') - 1] == ',') {
        out.push_back(""); // getline drops a trailing empty cell
    }
    return out;
}

std::string at(const std::vector<std::string>& header, const std::vector<std::string>& row,
               const char* column) {
    const auto it = std::find(header.begin(), header.end(), column);
    EXPECT_NE(it, header.end()) << column;
    return it == header.end() ? "?" : row[it - header.begin()];
}

TEST(LogRow, HeaderAndRowsLineUp) {
    const std::vector<std::string> header = cells(logHeader());
    EXPECT_EQ(header.front(), "t_us");
    EXPECT_EQ(header.back(), "set_heading");

    LogRow empty;
    std::string text;
    appendLogRow(empty, text);
    EXPECT_EQ(cells(text).size(), header.size()) << text;
    EXPECT_EQ(text.back(), '\n');

    LogRow full;
    full.sensors.imu = ImuSample{};
    for (auto& count : full.sensors.encoder_counts) {
        count = -1;
    }
    for (auto& position : full.sensors.motor_positions) {
        position = 1.0;
    }
    full.pose_set = true;
    text.clear();
    appendLogRow(full, text);
    EXPECT_EQ(cells(text).size(), header.size()) << text;
}

TEST(LogRow, WritesReadingsAndLeavesMissingOnesEmpty) {
    LogRow row;
    row.sensors.time_us           = 12345678901ull;
    row.sensors.board.port_open   = true;
    row.sensors.board.fresh       = true;
    row.sensors.board.restarted   = true;
    row.sensors.board.frames      = 500;
    row.sensors.board.dropped     = 2;
    row.sensors.encoder_counts[0] = -1520;
    row.sensors.encoder_counts[1] = 87;
    row.sensors.motor_positions[3] = 90.5;
    ImuSample imu;
    imu.rotation       = 3600.125;
    imu.gyro.z         = -1.5;
    imu.accel.z        = 1.0;
    row.sensors.imu    = imu;
    row.pose.x         = 7.5;
    row.pose.y         = 69.5;
    row.pose.heading   = 359.25;

    std::string text;
    appendLogRow(row, text);
    const std::vector<std::string> h = cells(logHeader());
    const std::vector<std::string> r = cells(text);

    EXPECT_EQ(at(h, r, "t_us"), "12345678901");
    EXPECT_EQ(at(h, r, "board_open"), "1");
    EXPECT_EQ(at(h, r, "board_restarted"), "1");
    EXPECT_EQ(at(h, r, "board_frames"), "500");
    EXPECT_EQ(at(h, r, "board_dropped"), "2");
    EXPECT_EQ(at(h, r, "tw0"), "-1520");
    EXPECT_EQ(at(h, r, "tw1"), "87");
    EXPECT_EQ(at(h, r, "tw2"), "");
    EXPECT_EQ(at(h, r, "m3"), "90.5000");
    EXPECT_EQ(at(h, r, "m4"), "");
    EXPECT_EQ(at(h, r, "imu_rotation"), "3600.1250");
    EXPECT_EQ(at(h, r, "imu_gyro_z"), "-1.5000");
    EXPECT_EQ(at(h, r, "imu_accel_z"), "1.0000");
    EXPECT_EQ(at(h, r, "x"), "7.5000");
    EXPECT_EQ(at(h, r, "heading"), "359.2500");
    EXPECT_EQ(at(h, r, "set_x"), "");

    LogRow no_imu;
    text.clear();
    appendLogRow(no_imu, text);
    EXPECT_EQ(at(h, cells(text), "imu_rotation"), "");
}

TEST(LogRow, PoseSetFillsTheSetColumns) {
    LogRow row;
    row.pose_set    = true;
    row.set.x       = 23.4;
    row.set.y       = 46.8;
    row.set.heading = 90;
    std::string text;
    appendLogRow(row, text);
    const std::vector<std::string> h = cells(logHeader());
    const std::vector<std::string> r = cells(text);
    EXPECT_EQ(at(h, r, "set_x"), "23.4000");
    EXPECT_EQ(at(h, r, "set_y"), "46.8000");
    EXPECT_EQ(at(h, r, "set_heading"), "90.0000");
}

TEST(LogQueue, InOrderAndDropsWhenFull) {
    LogQueue<int, 4> queue;
    int              out = 0;
    EXPECT_FALSE(queue.pop(out));
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(queue.push(i));
    }
    EXPECT_FALSE(queue.push(99));
    for (int i = 0; i < 4; ++i) {
        ASSERT_TRUE(queue.pop(out));
        EXPECT_EQ(out, i);
    }
    EXPECT_FALSE(queue.pop(out));
}

TEST(LogQueue, KeepsOrderAcrossManyLaps) {
    LogQueue<uint32_t, 8> queue;
    uint32_t              out = 0;
    for (uint32_t i = 0; i < 1000; ++i) {
        ASSERT_TRUE(queue.push(i));
        ASSERT_TRUE(queue.pop(out));
        EXPECT_EQ(out, i);
    }
}

TEST(LogQueue, TwoThreadsLoseNothingButWhatWasDropped) {
    LogQueue<uint32_t, 64> queue;
    constexpr uint32_t     kCount = 200000;
    uint32_t               dropped = 0;
    std::thread            writer([&] {
        for (uint32_t i = 0; i < kCount; ++i) {
            if (!queue.push(i)) {
                ++dropped;
            }
        }
    });
    uint32_t received = 0;
    uint32_t last     = 0;
    bool     ordered  = true;
    uint32_t out      = 0;
    while (true) {
        if (queue.pop(out)) {
            ordered = ordered && (received == 0 || out > last);
            last    = out;
            ++received;
            if (out == kCount - 1) {
                break;
            }
        } else if (received + dropped == kCount) {
            break; // the last one was dropped
        }
    }
    writer.join();
    while (queue.pop(out)) {
        ++received;
    }
    EXPECT_TRUE(ordered);
    EXPECT_EQ(received + dropped, kCount);
}

} // namespace
} // namespace locagatr
