// pipeline.h
// The path following loop. The Navigator task calls run() once per route, or
// runStep() once per step (pros/navigator.cpp). Portable: pose and time come
// in through Io.
//
//   Outcome Pipeline::run(route) {
//       plan(route);                                    // per move, before it drives:
//                                                       //   1. path     samplePath(segments)
//                                                       //   2. profile  profile_->build(path)
//       for (step : route)                              // then each step in order,
//           every tick until settled:                   // rate_hz times a second:
//               command = follower_->update(pose, target, dt);  // 3. follow
//               drive_->apply(command);                         // 4. drive
//   }
//
// Stages: path.h, profiles/profile.h, followers/follower.h, drives/drive.h.
// What runs in each: Config.

#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "navigatr/config.h"
#include "navigatr/drives/drive.h"
#include "navigatr/followers/follower.h"
#include "navigatr/path.h"
#include "navigatr/profiles/profile.h"
#include "navigatr/route.h"
#include "navigatr/types.h"

namespace navigatr
{

struct Io {
    std::function<Pose()>         pose;        // the localizer's newest pose
    std::function<uint32_t()>     now_ms;
    std::function<void(uint32_t)> sleep_until; // ms, as now_ms counts
    std::function<bool()>         cancelled;   // optional. True: stop the route
    std::function<void(double)>   progress;    // optional. Driving distance covered, 0 to 1
};

// Stages 1 and 2 for one move.
struct MovePlan {
    Path       path;
    Trajectory trajectory;
};

struct Plan {
    std::vector<MovePlan> moves;        // one per move step, in route order
    double                length   = 0; // in, every move
    double                duration = 0; // s, every move as profiled; turns and waits not counted
};

// How a run ended.
struct Outcome {
    bool     cancelled       = false;
    uint32_t timed_out       = 0;  // steps that gave up
    int      first_timed_out = -1; // index into Route::steps()

    bool ok() const { return !cancelled && timed_out == 0; }
};

class Pipeline {
  public:
    Pipeline(const Config& config, Io io);
    ~Pipeline();
    Pipeline(const Pipeline&)            = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    // False if a stage failed to begin or there is no drive.
    bool begin();

    // After begin().
    Plan plan(const Route& route) const;

    // Every step in order, in the calling task. A timed out step does not stop
    // the route; a cancel does.
    Outcome run(const Route& route);

    // One step of a route, using plan(route) made beforehand. Progress covers
    // this step only. The drive stops after it.
    Outcome runStep(const Route& route, const Plan& plan, std::size_t index);

  private:
    enum class End { kSettled, kTimedOut, kCancelled };

    End  step(const Step& s, const MovePlan* move_plan, double done, double total);
    static void record(End end, std::size_t index, Outcome& outcome);
    void report(double fraction);
    End  move(const MovePlan& plan, double done, double total);
    End  turn(const Step& step);
    End  wait(const Step& step);
    bool cancelled() const { return io_.cancelled && io_.cancelled(); }
    End track(const std::function<Target(double t, bool& arrived)>& target, double timeout_s,
              const std::function<void(double t)>& on_tick);

    Config                    config_;
    Io                        io_;
    std::unique_ptr<Profile>  own_profile_;
    std::unique_ptr<Follower> own_follower_;
    Profile*                  profile_  = nullptr;
    Follower*                 follower_ = nullptr;
    Drive*                    drive_    = nullptr;
};

} // namespace navigatr
