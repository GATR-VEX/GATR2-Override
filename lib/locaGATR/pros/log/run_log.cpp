// run_log.cpp
// PROS only.

#include "locagatr/log/run_log.h"

#include <cstdio>
#include <string>

#include "pros/misc.h"
#include "pros/rtos.hpp"

namespace locagatr
{

namespace
{

constexpr uint32_t kWriteMs  = 500;
constexpr int      kMaxFiles = 999;

bool exists(const char* path) {
    std::FILE* file = std::fopen(path, "r");
    if (file == nullptr) {
        return false;
    }
    std::fclose(file);
    return true;
}

// Open, write, close: the bytes are on the card once this returns.
bool write(const char* path, const std::string& text, const char* mode) {
    std::FILE* file = std::fopen(path, mode);
    if (file == nullptr) {
        return false;
    }
    const bool written = std::fwrite(text.data(), 1, text.size(), file) == text.size();
    return std::fclose(file) == 0 && written;
}

} // namespace

bool RunLog::start() {
    if (pros::c::usd_is_installed() != 1) {
        return false;
    }
    for (int n = 1; n <= kMaxFiles && number_ == 0; ++n) {
        std::snprintf(path_, sizeof path_, "/usd/run%03d.csv", n);
        if (!exists(path_)) {
            number_ = static_cast<uint16_t>(n);
        }
    }
    if (number_ == 0 || !write(path_, logHeader(), "w")) {
        return false;
    }
    writing_.store(true);
    if (pros::c::task_create(&RunLog::taskEntry, this, TASK_PRIORITY_MIN + 1,
                             TASK_STACK_DEPTH_DEFAULT, "locagatr log") == nullptr) {
        writing_.store(false);
        return false;
    }
    return true;
}

void RunLog::push(const LogRow& row) {
    if (writing_.load() && !queue_.push(row)) {
        dropped_.fetch_add(1);
    }
}

LogStatus RunLog::status() const {
    LogStatus s;
    s.writing = writing_.load();
    s.number  = number_;
    s.rows    = rows_.load();
    s.dropped = dropped_.load();
    return s;
}

void RunLog::run() {
    std::string text;
    LogRow      row;
    uint32_t    next = pros::millis();
    while (writing_.load()) {
        text.clear();
        uint32_t rows = 0;
        while (queue_.pop(row)) {
            appendLogRow(row, text);
            ++rows;
        }
        if (rows > 0 && !write(path_, text, "a")) {
            writing_.store(false); // card pulled or full
            break;
        }
        rows_.fetch_add(rows);
        pros::c::task_delay_until(&next, kWriteMs);
    }
    pros::c::task_delete(nullptr);
}

} // namespace locagatr
