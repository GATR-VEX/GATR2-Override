// report.cpp

#include "locagatr/report.h"

#include <cstdarg>
#include <cstdio>

namespace locagatr
{

namespace
{

void add(std::string& out, const char* format, ...) {
    char    buffer[96];
    va_list args;
    va_start(args, format);
    std::vsnprintf(buffer, sizeof buffer, format, args);
    va_end(args);
    out += buffer;
}

// "name: ok | name: FAILED | name: off"
template <typename T>
void stage(std::string& out, const char* label, const std::vector<Enabled<T>>& list,
           uint32_t running) {
    out += label;
    if (list.empty()) {
        out += "none";
    }
    for (std::size_t i = 0; i < list.size(); ++i) {
        const Enabled<T>& entry = list[i];
        const char* name  = entry.implementation != nullptr ? entry.implementation->name() : "(null)";
        const char* state = !entry.enabled ? "off" : ((running >> i) & 1u) != 0 ? "ok" : "FAILED";
        add(out, "%s%s: %s", i > 0 ? " | " : "", name, state);
    }
    out += '\n';
}

} // namespace

std::string describe(const Config& config, const Status& status) {
    std::string out;
    stage(out, "sources     ", config.sources, status.sources_running);
    stage(out, "processors  ", config.processors, status.processors_running);

    add(out, "estimator   %s: ", config.estimator != nullptr ? config.estimator->name() : "odometry");
    if (!status.estimator_running) {
        out += "FAILED\n";
    } else if (status.estimate.missing != nullptr) {
        add(out, "waiting for \"%s\"\n", status.estimate.missing);
    } else {
        out += status.estimate.tracking ? "tracking\n" : "not tracking\n";
    }

    const BoardStatus& board = status.sensors.board;
    add(out, "board       %s, %lu frames, %lu dropped, %lu unhandled records",
        !board.port_open ? "port not open" : board.fresh ? "streaming" : "not streaming",
        static_cast<unsigned long>(board.frames), static_cast<unsigned long>(board.dropped),
        static_cast<unsigned long>(board.unhandled_records));
    if (board.unhandled_records > 0) {
        add(out, " (last: type 0x%02X port %u)", board.last_unhandled_type,
            board.last_unhandled_port);
    }
    out += '\n';

    const LogStatus& log = status.log;
    out += "log         ";
    if (log.error != nullptr) {
        add(out, "FAILED: %s", log.error);
    } else if (log.writing) {
        out += "writing";
    } else {
        out += log.number == 0 ? "off" : "stopped";
    }
    if (log.number != 0) {
        add(out, ", /usd/run%03u.csv, %lu rows, %lu dropped", static_cast<unsigned>(log.number),
            static_cast<unsigned long>(log.rows), static_cast<unsigned long>(log.dropped));
    }
    out += '\n';

    out += "motion      ";
    if (status.motion.count == 0) {
        out += "none";
    }
    for (std::size_t i = 0; i < status.motion.count; ++i) {
        const Measurement& m = status.motion.entries[i];
        add(out, "%s%s", i > 0 ? " | " : "", m.name);
        if (m.forward) {
            add(out, " fwd %.2f", *m.forward);
        }
        if (m.lateral) {
            add(out, " lat %.2f", *m.lateral);
        }
        if (m.yaw) {
            add(out, " yaw %.1f", *m.yaw);
        }
        if (m.pitch) {
            add(out, " pitch %.1f", *m.pitch);
        }
        if (m.roll) {
            add(out, " roll %.1f", *m.roll);
        }
    }
    out += '\n';

    const Pose& pose = status.estimate.pose;
    add(out, "pose        x %.2f y %.2f heading %.1f\n", pose.x, pose.y, pose.heading);
    add(out, "ticks       %lu, %lu overruns\n", static_cast<unsigned long>(status.ticks),
        static_cast<unsigned long>(status.overruns));
    return out;
}

} // namespace locagatr
