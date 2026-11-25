// run_log.h
// Config::log: every tick to /usd/runNNN.csv on the microSD card, a new file
// each program run. The Localizer task pushes rows; a low priority task
// writes them twice a second, so a power cut loses about the last 0.5 s.
// Source in pros/log/run_log.cpp.

#pragma once
#include <atomic>
#include <cstdint>

#include "locagatr/log/log_queue.h"
#include "locagatr/log/log_row.h"

namespace locagatr
{

struct LogStatus {
    bool     writing = false; // file open, rows going in
    uint16_t number  = 0;     // file /usd/run<number>.csv, 0 before one is picked
    uint32_t rows    = 0;     // written
    uint32_t dropped = 0;     // lost because the card fell behind
};

class RunLog {
  public:
    // False: no SD card, no free file name, or the task did not start.
    bool start();

    // Never waits.
    void push(const LogRow& row);

    LogStatus status() const;

  private:
    static void taskEntry(void* self) { static_cast<RunLog*>(self)->run(); }
    void        run();

    static constexpr std::size_t kRows = 256; // 2.5 s at 100 Hz

    LogQueue<LogRow, kRows> queue_;
    char                    path_[24] = {};
    uint16_t                number_   = 0;
    std::atomic<bool>       writing_{false};
    std::atomic<uint32_t>   rows_{0};
    std::atomic<uint32_t>   dropped_{0};
};

} // namespace locagatr
