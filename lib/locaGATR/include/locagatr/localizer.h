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
#include "locagatr/log/log_status.h"
#include "locagatr/pipeline.h"
#include "locagatr/slot.h"

namespace locagatr
{

class RunLog;

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
    LogStatus log;                    // startLog()
};

class Localizer {
  public:
    explicit Localizer(const Config& config);
    ~Localizer();
    Localizer(const Localizer&)            = delete;
    Localizer& operator=(const Localizer&) = delete;

    // From initialize(). Blocks while the IMU calibrates. False if anything
    // failed to begin; the rest still run.
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

    // Every tick from now on to a new file on the microSD card,
    // /usd/runNNN.csv; see log/run_log.h. After start(). False: no card, or
    // the file could not be made (getStatus().log.error says which).
    bool startLog();
    // Writes the rest and closes the file. Returns once it is complete.
    void stopLog();
    bool isLogging() const;

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

    Config                  config_;
    Pipeline                pipeline_;
    Slot<Snapshot>          snapshot_;
    std::unique_ptr<RunLog> log_; // made in start()
    Slot<Pose>              pose_request_;
    std::atomic<uint32_t>   pose_applied_{0}; // pose_request_ version published
    std::atomic<int>        started_{0};      // 0 not yet, 1 started, -1 failed
    bool                    start_ok_ = false;
};

} // namespace locagatr
