// navigator.h
// Runs routes in its own task, reading the pose from a Localizer. A route
// stops on cancel() or when the competition mode changes. Source in pros/.
//
//   navigator.follow(route);      // blocks
//
//   navigator.start(route);       // or returns at once
//   navigator.waitUntil(0.5);     // half the driving distance
//   navigator.wait();
//
//   navigator.load(route);        // or one step at a time
//   navigator.next();             // the first move, turn or wait
//   navigator.wait();
//   score();                      // nothing moving
//   navigator.next();             // false after the last step

#pragma once
#include <atomic>

#include "locagatr/localizer.h"
#include "navigatr/config.h"
#include "navigatr/pipeline.h"
#include "navigatr/route.h"

namespace navigatr
{

class Navigator {
  public:
    Navigator(const Config& config, const locagatr::Localizer& localizer);
    Navigator(const Navigator&)            = delete;
    Navigator& operator=(const Navigator&) = delete;

    // Returns at once. False if a route is running or the stages failed to begin.
    bool start(const Route& route);

    // Plans the route for next(). False if a route is running or the stages
    // failed to begin.
    bool load(const Route& route);
    // Starts the next step of the loaded route and returns at once. False
    // after the last step, with nothing loaded, or while a step is running.
    bool next();

    // True if every step settled.
    bool wait();
    // Fraction of the driving distance, of the route or of the step next()
    // started. False if it ended first.
    bool waitUntil(double fraction);
    bool follow(const Route& route);

    void cancel();

    bool   isRunning() const { return running_.load(); }
    double progress() const { return progress_.load(); }
    // The last route or step, once done.
    Outcome outcome() const { return outcome_; }

  private:
    static void taskEntry(void* self) { static_cast<Navigator*>(self)->run(); }
    void        run();
    bool        ready();  // stages begun, task created
    void        launch(); // wakes the task for route_

    Pipeline           pipeline_;
    Route              route_{Pose{}};
    Plan               plan_;          // load(): the loaded route
    bool               loaded_ = false;
    std::size_t        next_   = 0;    // load(): step next() runs
    int                step_   = -1;   // the step running; -1: the whole route
    Outcome            outcome_;
    void*              task_  = nullptr;
    int                begun_ = 0; // 0 not yet, 1 ok, -1 failed
    std::atomic<bool>  running_{false};
    std::atomic<bool>  cancel_{false};
    std::atomic<float> progress_{0};
    std::atomic<int>   mode_{0}; // competition mode when the route started
};

} // namespace navigatr
