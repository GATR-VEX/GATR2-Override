// display.cpp
// The pose on the Brain screen and in spectaGATR.

#include "display.h"

#include <cstdio>
#include <optional>

#include "localization.h"
#include "main.h"

namespace
{

constexpr uint32_t kPeriodMs = 50;

spectagatr::ViewerConfig vizConfig() {
    spectagatr::ViewerConfig c;
    // Chassis 36.8 cm long, 37.4 cm wide, centered on the tracked point.
    c.robot = spectagatr::RobotBox::centered(0.368, 0.374, 0.38); // height: CHECK
    return c;
}

void count(char* text, std::size_t size, const char* name, const std::optional<int32_t>& value) {
    if (value) {
        std::snprintf(text, size, "%s %ld", name, static_cast<long>(*value));
    } else {
        std::snprintf(text, size, "%s --", name);
    }
}

// Padded so a shorter line covers the last one.
void line(int n, const char* text) {
    pros::screen::print(pros::E_TEXT_MEDIUM, n, "%-48s", text);
}

void run(void*) {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    uint32_t         tick = 0;
    char             text[96];
    while (true) {
        const locagatr::Pose   pose   = localizer.getPose();
        const locagatr::Status status = localizer.getStatus();
        const locagatr::SensorMap&   sensors  = status.sensors;
        const locagatr::BoardStatus& board    = sensors.board;
        const locagatr::Estimate&    estimate = status.estimate;

        viz.updatePose(spectagatr::inchesToMeters(pose.x), spectagatr::inchesToMeters(pose.y),
                       spectagatr::degreesToRadians(pose.heading));

        std::snprintf(text, sizeof text, "x %.1f  y %.1f  h %.1f", pose.x, pose.y, pose.heading);
        line(0, text);

        char tw[3][24];
        count(tw[0], sizeof tw[0], "TW0", sensors.encoder_counts[0]);
        count(tw[1], sizeof tw[1], "TW1", sensors.encoder_counts[1]);
        count(tw[2], sizeof tw[2], "TW2", sensors.encoder_counts[2]);
        std::snprintf(text, sizeof text, "%s  %s  %s", tw[0], tw[1], tw[2]);
        line(1, text);

        const char* link = !board.port_open ? "NO PORT" : board.fresh ? "ok" : "QUIET";
        std::snprintf(text, sizeof text, "board %s  frames %lu  dropped %lu", link,
                      static_cast<unsigned long>(board.frames),
                      static_cast<unsigned long>(board.dropped));
        line(2, text);

        if (sensors.imu) {
            std::snprintf(text, sizeof text, "imu %.1f", sensors.imu->rotation);
        } else {
            std::snprintf(text, sizeof text, "imu --");
        }
        line(3, text);

        std::snprintf(text, sizeof text, "%s%s", estimate.missing != nullptr ? "missing " : "tracking",
                      estimate.missing != nullptr ? estimate.missing : "");
        line(4, text);

        // Controller: one row per tick, it takes no more than one print per 50 ms.
        switch (tick++ % 3) {
        case 0: controller.print(0, 0, "x%6.1f y%6.1f   ", pose.x, pose.y); break;
        case 1:
            controller.print(1, 0, "h%6.1f %-8s", pose.heading,
                             !board.port_open ? "NO PORT" : board.fresh ? "board ok" : "NO BOARD");
            break;
        case 2:
            controller.print(2, 0, "%-17s", estimate.missing != nullptr ? estimate.missing : "tracking");
            break;
        }

        pros::delay(kPeriodMs);
    }
}

} // namespace

spectagatr::Viewer viz(vizConfig());

void message(int n, const char* text) { line(n, text); }

void startDisplay() {
    viz.start();
    pros::c::task_create(&run, nullptr, TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT,
                         "display");
}
