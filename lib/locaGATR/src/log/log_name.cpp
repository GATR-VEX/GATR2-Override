// log_name.cpp

#include "locagatr/log/log_name.h"

#include <cctype>
#include <cstdio>
#include <cstring>

namespace locagatr
{

namespace
{

bool sameIgnoringCase(const std::string& a, std::size_t at, const char* b, std::size_t length) {
    for (std::size_t i = 0; i < length; ++i) {
        if (std::tolower(static_cast<unsigned char>(a[at + i])) !=
            std::tolower(static_cast<unsigned char>(b[i]))) {
            return false;
        }
    }
    return true;
}

// The number in "<name><digits>.csv", or 0.
int numberOf(const std::string& file, const char* name) {
    const std::size_t name_length = std::strlen(name);
    if (file.size() < name_length + 5 || !sameIgnoringCase(file, 0, name, name_length) ||
        !sameIgnoringCase(file, file.size() - 4, ".csv", 4)) {
        return 0;
    }
    const std::size_t digits = file.size() - name_length - 4;
    if (digits > 6) {
        return 0;
    }
    int number = 0;
    for (std::size_t i = name_length; i < name_length + digits; ++i) {
        if (!std::isdigit(static_cast<unsigned char>(file[i]))) {
            return 0;
        }
        number = number * 10 + (file[i] - '0');
    }
    return number;
}

} // namespace

bool validLogName(const char* name) {
    if (name == nullptr) {
        return false;
    }
    const std::size_t length = std::strlen(name);
    if (length == 0 || length > kMaxLogName) {
        return false;
    }
    for (std::size_t i = 0; i < length; ++i) {
        const unsigned char c = static_cast<unsigned char>(name[i]);
        if (!std::isalnum(c) && c != '_' && c != '-') {
            return false;
        }
    }
    return true;
}

int nextLogNumber(const std::string& listing, const char* name) {
    int         highest = 0;
    std::size_t start   = 0;
    while (start < listing.size()) {
        std::size_t end = listing.find('\n', start);
        if (end == std::string::npos) {
            end = listing.size();
        }
        std::string file = listing.substr(start, end - start);
        if (!file.empty() && file.back() == '\r') {
            file.pop_back();
        }
        const int number = numberOf(file, name);
        if (number > highest) {
            highest = number;
        }
        start = end + 1;
    }
    return highest + 1;
}

std::string logFileName(const char* name, int number) {
    char text[kMaxLogName + 16];
    std::snprintf(text, sizeof text, "%s%03d.csv", name, number);
    return text;
}

} // namespace locagatr
