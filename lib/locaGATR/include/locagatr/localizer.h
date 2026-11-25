// localizer.h
// The robot pose, from one task running the pipeline. Source in pros/.
//
//   locagatr::Localizer localizer(config());
//   void initialize() { localizer.start(); }
//   double x = localizer.getX();

#pragma once
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "locagatr/config.h"
#include "locagatr/log/run_log.h"
#include "locagatr/pipeline.h"
#include "locagatr/slot.h"

namespace locagatr
{

// The last tick.
struct Status {
    SensorMap sensors;                // raw readings
    MotionMap motion;                 // every measurement
    Estimate  estimate;               // the published pose
    uint32_t  sources_running    = 0; // bit i: Config::sources[i] began and runs
    uint32_t  processors_running = 0; // bit i: Config::processors[i]
    bool      estimator_running  = false;
    uint32_t  ticks              = 0;
    uint32_t  overruns           = 0; // ticks that took longer than 1 / rate_hz
    LogStatus log;                    // Config::log
};

class Localizer {
  public:
    explicit Localizer(const Config& config) : config_(config), pipeline_(config) {}
    Localizer(const Localizer&)            = delete;
    Localizer& operator=(const Localizer&) = delete;

    // From initialize(). Blocks while the IMU calibrates. False if anything
    // failed to begin, the log included; the rest still run.
    bool start();

    Pose   getPose() const;
    double getX() const;       // in
    double getY() const;       // in
    double getHeading() const; // deg counterclockwise from +x, [0, 360)
    double getPitch() const;   // deg
    double getRoll() const;    // deg

    // Waits about one tick, until getPose() returns it. Pitch and roll are not set.
    void setPose(const Pose& pose);
    void setPose(double x, double y, double heading);

    Status getStatus() const;

    // Status as text, see report.h.
    std::string describe() const;

  private:
    struct Snapshot {
        Pose   pose;
        Status status;
    };

    static void taskEntry(void* self) { static_cast<Localizer*>(self)->run(); }
    void        run();
    Snapshot    snapshot() const;

    Config           config_;
    Pipeline         pipeline_;
    Slot<Snapshot>   snapshot_;
    std::unique_ptr<RunLog> log_; // made in start() when Config::log
    Slot<Pose>            pose_request_;
    std::atomic<uint32_t> pose_applied_{0}; // pose_request_ version published
    std::atomic<int>      started_{0};      // 0 not yet, 1 started, -1 failed
    bool             start_ok_ = false;
};

} // namespace locagatr
