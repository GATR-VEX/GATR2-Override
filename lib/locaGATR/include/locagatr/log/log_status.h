// log_status.h
// What the run log is doing, in Status::log. Portable.

#pragma once
#include <cstdint>

namespace locagatr
{

struct LogStatus {
    bool        writing  = false;   // between startLog() and stopLog()
    char        file[32] = {};      // the last file, under /usd/; empty: none yet
    uint32_t    rows     = 0;       // written to it
    uint32_t    dropped  = 0;       // lost because the card fell behind
    const char* error    = nullptr; // why the last start or write failed
};

} // namespace locagatr
