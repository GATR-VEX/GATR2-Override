// run_log.cpp
// PROS only.

#include "locagatr/log/run_log.h"

#include <cstdio>

#include "pros/misc.h"

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
    mutex_.take();
    if (writing_.load()) {
        mutex_.give();
        return true;
    }
    error_.store(nullptr);
    if (pros::c::usd_is_installed() != 1) {
        return fail("no SD card");
    }

    int number = 0;
    for (int n = 1; n <= kMaxFiles && number == 0; ++n) {
        std::snprintf(path_, sizeof path_, "/usd/run%03d.csv", n);
        if (!exists(path_)) {
            number = n;
        }
    }
    if (number == 0) {
        return fail("run001 to run999 all used");
    }
    if (!write(path_, logHeader(), "w")) {
        return fail("could not create the file");
    }
    if (!task_started_) {
        task_started_ = pros::c::task_create(&RunLog::taskEntry, this, TASK_PRIORITY_MIN + 1,
                                             TASK_STACK_DEPTH_DEFAULT, "locagatr log") != nullptr;
        if (!task_started_) {
            return fail("no task");
        }
    }

    LogRow stale; // pushed while the last file was closing
    while (queue_.pop(stale)) {
    }
    number_.store(static_cast<uint16_t>(number));
    rows_.store(0);
    dropped_.store(0);
    writing_.store(true);
    mutex_.give();
    return true;
}

void RunLog::stop() {
    mutex_.take();
    if (writing_.load()) {
        writing_.store(false);
        flush();
    }
    mutex_.give();
}

void RunLog::push(const LogRow& row) {
    if (writing_.load() && !queue_.push(row)) {
        dropped_.fetch_add(1);
    }
}

LogStatus RunLog::status() const {
    LogStatus s;
    s.writing = writing_.load();
    s.number  = number_.load();
    s.rows    = rows_.load();
    s.dropped = dropped_.load();
    s.error   = error_.load();
    return s;
}

bool RunLog::fail(const char* error) {
    error_.store(error);
    mutex_.give();
    return false;
}

void RunLog::flush() {
    text_.clear();
    uint32_t rows = 0;
    LogRow   row;
    while (queue_.pop(row)) {
        appendLogRow(row, text_);
        ++rows;
    }
    if (rows == 0) {
        return;
    }
    if (write(path_, text_, "a")) {
        rows_.fetch_add(rows);
    } else {
        writing_.store(false); // card pulled or full
        error_.store("a write failed");
    }
}

void RunLog::run() {
    uint32_t next = pros::millis();
    while (true) {
        mutex_.take();
        if (writing_.load()) {
            flush();
        }
        mutex_.give();
        pros::c::task_delay_until(&next, kWriteMs);
    }
}

} // namespace locagatr
