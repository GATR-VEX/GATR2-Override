// source.h
// Stage 1, acquisition: Source[] -> SensorMap, raw readings. Portable.

#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "locagatr/config.h"
#include "locagatr/types.h"

namespace locagatr
{

// Control board connectors a reading can come from (0 = TW0).
constexpr std::size_t kBoardPorts = 8;
// Smart Ports, indexed 1..21.
constexpr std::size_t kSmartPorts = 22;

struct ImuSample {
    double  rotation = 0; // deg counterclockwise since calibration, unwrapped
    double  pitch    = 0; // deg
    double  roll     = 0; // deg
    Vector3 gyro;         // deg/s, sensor axes as PROS reports them
    Vector3 accel;        // g, sensor axes as PROS reports them
};

struct BoardStatus {
    bool     port_open           = false; // the Smart Port is open; retried until it is
    bool     fresh               = false; // frames are arriving
    bool     restarted           = false; // its counts started over this tick
    uint32_t frames              = 0;     // good frames, since start
    uint32_t dropped             = 0;     // frames lost on the way, since start
    uint32_t unhandled_records   = 0; // records with no decoder, since start
    uint8_t  last_unhandled_type = 0;
    uint8_t  last_unhandled_port = 0;
};

// Every raw reading this tick, each source filling its own fields.
struct SensorMap {
    uint64_t    time_us = 0;
    double      dt      = 0; // s since the previous tick; 0 on the first
    BoardStatus board;
    std::array<std::optional<int32_t>, kBoardPorts> encoder_counts{};  // by board connector
    std::array<std::optional<double>, kSmartPorts>  motor_positions{}; // deg of the motor output, by port
    std::optional<ImuSample>                         imu;
};

class Source {
  public:
    virtual ~Source() = default;
    virtual const char* name() const = 0;
    // From Localizer::start(). False: left out.
    virtual bool begin(const Config&) { return true; }
    virtual void read(SensorMap& sensors) = 0;
};

} // namespace locagatr
