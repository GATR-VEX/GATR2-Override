// log_name.h
// Log file names: <name><number>.csv, the number one past the highest already
// on the card for that name. Portable.

#pragma once
#include <cstddef>
#include <string>

namespace locagatr
{

constexpr std::size_t kMaxLogName = 16;

// 1 to kMaxLogName letters, digits, '_' or '-'.
bool validLogName(const char* name);

// One past the highest <name><digits>.csv in listing (file names separated by
// newlines), ignoring case. 1 if there is none.
int nextLogNumber(const std::string& listing, const char* name);

// "<name><number, 3 digits at least>.csv"
std::string logFileName(const char* name, int number);

} // namespace locagatr
