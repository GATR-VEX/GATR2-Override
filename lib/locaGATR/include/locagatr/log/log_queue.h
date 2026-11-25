// log_queue.h
// Fixed size queue from one task to one other, no lock. push() never waits:
// when full it drops the value. Portable.

#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace locagatr
{

template <typename T, std::size_t N> class LogQueue {
    static_assert(N >= 2 && (N & (N - 1)) == 0, "N is a power of two");

  public:
    // From the one writing task. False: full, value dropped.
    bool push(const T& value) {
        const uint32_t head = head_.load(std::memory_order_relaxed);
        if (head - tail_.load(std::memory_order_acquire) >= N) {
            return false;
        }
        items_[head % N] = value;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    // From the one reading task. False: empty.
    bool pop(T& out) {
        const uint32_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        out = items_[tail % N];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

  private:
    std::array<T, N>      items_{};
    std::atomic<uint32_t> head_{0}; // pushes so far
    std::atomic<uint32_t> tail_{0}; // pops so far
};

} // namespace locagatr
