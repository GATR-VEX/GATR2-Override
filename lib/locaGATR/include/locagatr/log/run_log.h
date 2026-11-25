// run_log.h
// Localizer::startLog(name) to stopLog(): every tick to /usd/<name>NNN.csv
// on the microSD card, a new file each start. The Localizer task pushes rows;
// a low priority task writes them twice a second, opening and closing the file
// each time, so a program stopped mid log loses about the last 0.5 s and
// nothing else. PROS only: source in pros/log/run_log.cpp.

#pragma once
#include <atomic>
#include <cstdint>
#include <string>

#include "locagatr/log/log_queue.h"
#include "locagatr/log/log_row.h"
#include "locagatr/log/log_status.h"
#include "locagatr/slot.h"
#include "pros/rtos.hpp"

namespace locagatr
{

class RunLog {
  public:
    // A new file, see log_name.h. False: bad name, no SD card, or the file
    // could not be made. Already writing: keeps that file, true.
    bool start(const char* name);

    // Writes what is queued and closes the file. Returns once it is complete.
    void stop();

    // From the Localizer task, every tick. Never waits.
    void push(const LogRow& row);

    LogStatus status() const;

  private:
    struct FileName {
        char text[sizeof(LogStatus::file)] = {};
    };

    static void taskEntry(void* self) { static_cast<RunLog*>(self)->run(); }
    void        run();
    bool        fail(const char* error);
    void        flush(); // with mutex_ held

    static constexpr std::size_t kRows = 256; // 2.5 s at 100 Hz

    LogQueue<LogRow, kRows>  queue_;
    pros::Mutex              mutex_; // the file and the reading end of queue_
    std::string              text_;
    std::string              path_;
    Slot<FileName>           file_;
    bool                     task_started_ = false;
    std::atomic<bool>        writing_{false};
    std::atomic<uint32_t>    rows_{0};
    std::atomic<uint32_t>    dropped_{0};
    std::atomic<const char*> error_{nullptr};
};

} // namespace locagatr
