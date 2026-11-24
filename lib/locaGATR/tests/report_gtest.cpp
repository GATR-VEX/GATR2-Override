// report_gtest.cpp

#include "locagatr/report.h"

#include <gtest/gtest.h>

namespace locagatr
{
namespace
{

struct Named : Source {
    explicit Named(const char* n) : n(n) {}
    const char* name() const override { return n; }
    void        read(SensorMap&) override {}
    const char* n;
};

struct NamedProcessor : Processor {
    const char* name() const override { return "IMU"; }
    void        process(const SensorMap&, MotionMap&) override {}
};

bool has(const std::string& text, const char* part) { return text.find(part) != std::string::npos; }

TEST(Report, SaysWhatRunsFailedAndIsOff) {
    Named          imu("VEX IMU"), board("control board"), motors("motor encoders");
    NamedProcessor processor;
    Config         c;
    c.sources    = {{&imu, true}, {&board, true}, {&motors, false}};
    c.processors = {{&processor, true}};

    Status s;
    s.sources_running    = 0b001; // the board failed to begin
    s.processors_running = 0b1;
    s.estimator_running  = true;
    s.estimate.missing   = "back";
    Measurement* m       = s.motion.add("IMU");
    m->yaw               = 91.2;
    s.estimate.pose.x    = 12.5;
    EXPECT_TRUE(has(describe(c, s), "port not open"));
    s.sensors.board.port_open = true;

    const std::string text = describe(c, s);
    EXPECT_TRUE(has(text, "VEX IMU: ok | control board: FAILED | motor encoders: off")) << text;
    EXPECT_TRUE(has(text, "processors  IMU: ok")) << text;
    EXPECT_TRUE(has(text, "estimator   odometry: waiting for \"back\"")) << text;
    EXPECT_TRUE(has(text, "not answering")) << text;
    EXPECT_TRUE(has(text, "IMU yaw 91.2")) << text;
    EXPECT_TRUE(has(text, "x 12.50")) << text;
}

TEST(Report, EstimatorStates) {
    Status s;
    EXPECT_TRUE(has(describe(Config{}, s), "odometry: FAILED"));
    s.estimator_running = true;
    EXPECT_TRUE(has(describe(Config{}, s), "odometry: not tracking"));
    s.estimate.tracking = true;
    EXPECT_TRUE(has(describe(Config{}, s), "odometry: tracking"));
}

} // namespace
} // namespace locagatr
