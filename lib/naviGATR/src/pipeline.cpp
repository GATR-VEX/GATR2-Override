// pipeline.cpp
// The path following loop: plan the route, then each step.

#include "navigatr/pipeline.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "navigatr/followers/pure_pursuit_follower.h"
#include "navigatr/profiles/trapezoid_profile.h"

namespace navigatr
{

Pipeline::Pipeline(const Config& config, Io io) : config_(config), io_(std::move(io)) {}

Pipeline::~Pipeline() = default;

bool Pipeline::begin() {
    profile_ = config_.profile;
    if (profile_ == nullptr) {
        own_profile_ = std::make_unique<TrapezoidProfile>();
        profile_     = own_profile_.get();
    }
    follower_ = config_.follower;
    if (follower_ == nullptr) {
        own_follower_ = std::make_unique<PurePursuitFollower>();
        follower_     = own_follower_.get();
    }
    drive_ = config_.drive;
    return drive_ != nullptr && profile_->begin(config_) && follower_->begin(config_) &&
           drive_->begin(config_);
}

Plan Pipeline::plan(const Route& route) const {
    Plan plan;
    if (profile_ == nullptr) {
        return plan;
    }
    for (const Step& step : route.steps()) {
        if (step.kind != Step::Kind::kMove) {
            continue;
        }
        MovePlan move;
        // 1. Path.
        move.path = samplePath(step.segments);
        // 2. Profile.
        move.trajectory = profile_->build(move.path);

        plan.length += move.path.length();
        plan.duration += move.trajectory.duration();
        plan.moves.push_back(std::move(move));
    }
    return plan;
}

Outcome Pipeline::run(const Route& route) {
    const Plan  whole = plan(route);
    Outcome     outcome;
    std::size_t next_move = 0;
    double      done      = 0; // in, moves finished
    report(0.0);
    for (std::size_t i = 0; i < route.steps().size() && !outcome.cancelled; ++i) {
        const Step&     s         = route.steps()[i];
        const MovePlan* move_plan = s.kind == Step::Kind::kMove ? &whole.moves[next_move++] : nullptr;
        record(step(s, move_plan, done, whole.length), i, outcome);
        if (move_plan != nullptr) {
            done += move_plan->path.length();
        }
    }
    drive_->stop();
    if (!outcome.cancelled) {
        report(1.0);
    }
    return outcome;
}

Outcome Pipeline::runStep(const Route& route, const Plan& plan, std::size_t index) {
    Outcome outcome;
    if (index >= route.steps().size()) {
        return outcome;
    }
    std::size_t move_index = 0; // moves before this step
    for (std::size_t i = 0; i < index; ++i) {
        move_index += route.steps()[i].kind == Step::Kind::kMove ? 1 : 0;
    }
    const Step&     s         = route.steps()[index];
    const MovePlan* move_plan = nullptr;
    if (s.kind == Step::Kind::kMove) {
        if (move_index >= plan.moves.size()) {
            return outcome; // plan is not from this route
        }
        move_plan = &plan.moves[move_index];
    }
    report(0.0);
    const double length = move_plan != nullptr ? move_plan->path.length() : 0.0;
    record(step(s, move_plan, 0, length), index, outcome);
    drive_->stop();
    if (!outcome.cancelled) {
        report(1.0);
    }
    return outcome;
}

Pipeline::End Pipeline::step(const Step& s, const MovePlan* move_plan, double done, double total) {
    if (cancelled()) {
        return End::kCancelled;
    }
    switch (s.kind) {
    case Step::Kind::kMove: return move(*move_plan, done, total);
    case Step::Kind::kTurn: return turn(s);
    case Step::Kind::kWait: return wait(s);
    }
    return End::kCancelled;
}

void Pipeline::record(End end, std::size_t index, Outcome& outcome) {
    if (end == End::kCancelled) {
        outcome.cancelled = true;
    } else if (end == End::kTimedOut) {
        ++outcome.timed_out;
        if (outcome.first_timed_out < 0) {
            outcome.first_timed_out = static_cast<int>(index);
        }
    }
}

void Pipeline::report(double fraction) {
    if (io_.progress) {
        io_.progress(fraction);
    }
}

Pipeline::End Pipeline::move(const MovePlan& plan, double done, double total) {
    const Path&       path       = plan.path;
    const Trajectory& trajectory = plan.trajectory;

    std::size_t marker = 0;
    const auto  fire   = [&](double s) {
        while (marker < path.markers.size() && path.markers[marker].s <= s + 1e-9) {
            (*path.markers[marker++].run)();
        }
    };
    const End end = track(
        [&](double t, bool& arrived) {
            const TrajectoryPoint p = trajectory.at(t);
            arrived                 = t >= trajectory.duration();
            Goal goal;
            goal.target.x         = p.x;
            goal.target.y         = p.y;
            goal.target.heading   = p.heading;
            goal.target.vx        = p.v * std::cos(radians(p.tangent));
            goal.target.vy        = p.v * std::sin(radians(p.tangent));
            goal.target.turn_rate = p.heading_rate;
            goal.path             = &path;
            goal.trajectory       = &trajectory;
            goal.t                = t;
            return goal;
        },
        trajectory.duration() + config_.tolerances.timeout_ms / 1000.0,
        [&](double t) {
            const double s = trajectory.at(t).s;
            fire(s);
            if (io_.progress && total > 0) {
                io_.progress(std::min((done + s) / total, 1.0));
            }
        });
    if (end != End::kCancelled) {
        fire(path.length()); // whatever a timeout left
    }
    return end;
}

Pipeline::End Pipeline::turn(const Step& step) {
    for (const Action& action : step.actions) {
        action.run();
    }
    const double angle = std::fabs(wrap180(step.heading - io_.pose().heading));
    const double limit = config_.limits.max_turn_rate > 0 ? config_.limits.max_turn_rate : 180.0;
    Goal         goal;
    goal.target.x       = step.position.x;
    goal.target.y       = step.position.y;
    goal.target.heading = step.heading;
    return track(
        [&](double t, bool& arrived) {
            arrived = true;
            goal.t  = t;
            return goal;
        },
        angle / limit + config_.tolerances.timeout_ms / 1000.0, [](double) {});
}

Pipeline::End Pipeline::wait(const Step& step) {
    for (const Action& action : step.actions) {
        action.run();
    }
    drive_->stop();
    const uint32_t period_ms = std::max<uint32_t>(1, 1000 / std::max<uint32_t>(1, config_.rate_hz));
    const uint32_t start     = io_.now_ms();
    while (true) {
        const uint32_t elapsed = io_.now_ms() - start;
        if (elapsed >= step.ms) {
            return End::kSettled;
        }
        if (cancelled()) {
            return End::kCancelled;
        }
        io_.sleep_until(start + elapsed + std::min(step.ms - elapsed, period_ms));
    }
}

Pipeline::End Pipeline::track(const std::function<Goal(double t, bool& arrived)>& goal_at,
                              double                                              timeout_s,
                              const std::function<void(double t)>&                on_tick) {
    follower_->reset();
    const Tolerances& tolerance = config_.tolerances;
    const uint32_t    period_ms = std::max<uint32_t>(1, 1000 / std::max<uint32_t>(1, config_.rate_hz));
    const double      dt        = period_ms / 1000.0;
    const uint32_t    start     = io_.now_ms();
    uint32_t          next      = start;
    uint32_t          settled   = 0;
    while (true) {
        if (cancelled()) {
            return End::kCancelled;
        }
        const double t = (io_.now_ms() - start) / 1000.0;
        on_tick(t);

        bool       arrived = false; // the target has stopped moving
        const Goal goal    = goal_at(t, arrived);
        const Pose pose    = io_.pose();
        // 3. Follow.
        const ChassisSpeeds command = follower_->update(pose, goal, dt);
        // 4. Drive.
        drive_->apply(command);

        if (arrived) {
            const Target& end = goal.target;
            const bool    close =
                std::hypot(end.x - pose.x, end.y - pose.y) <= tolerance.position &&
                std::fabs(wrap180(end.heading - pose.heading)) <= tolerance.heading;
            settled = close ? settled + period_ms : 0;
            if (settled >= tolerance.settle_ms) {
                return End::kSettled;
            }
        }
        if (t >= timeout_s) {
            return End::kTimedOut;
        }
        next += period_ms;
        io_.sleep_until(next);
    }
}

} // namespace navigatr
