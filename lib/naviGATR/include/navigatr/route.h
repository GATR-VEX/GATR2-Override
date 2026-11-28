// route.h
// An autonomous route. Segments in a row are one move, with no stop between
// them; path() starts a new one. The heading holds unless facing() or
// tangent() is given. Each move can pick its follower. Portable.
//
//   Route({24, 12, 90})
//       .bezier({36, 24}, {48, 48}, {60, 48}).facing(0)
//       .line({60, 72}).at(0.5, [] { intake.move(127); })
//       .path().follow(Follow::kTrajectory)
//       .line({36, 72})
//       .turnTo(180)
//       .wait(300);

#pragma once
#include <cstdint>
#include <functional>
#include <vector>

#include "navigatr/bezier.h"
#include "navigatr/types.h"

namespace navigatr
{

struct Action {
    double                at = 1.0; // along its segment, 0 to 1
    std::function<void()> run;      // runs in the following task; keep it short
};

struct Segment {
    Bezier              curve;
    double              start_heading = 0; // deg
    double              end_heading   = 0; // deg, reached at the segment's end
    bool                tangent       = false; // face along the path instead
    std::vector<Action> actions;
};

// Which follower drives a move.
enum class Follow {
    kDefault,     // Config::follower, else pure pursuit
    kPurePursuit, // PurePursuitFollower: toward a point ahead on the path
    kTrajectory,  // TrajectoryFollower: the trajectory point at time t
};

struct Step {
    enum class Kind { kMove, kTurn, kWait };
    Kind                 kind = Kind::kMove;
    std::vector<Segment> segments;                  // kMove
    Follow               follow = Follow::kDefault; // kMove
    Point                position; // kTurn: where to turn
    double               heading = 0; // kTurn: deg to face
    uint32_t             ms      = 0; // kWait
    std::vector<Action>  actions;     // kTurn, kWait: run when the step starts
};

class Route {
  public:
    explicit Route(const Pose& start);

    // The next segment starts a new move: the robot stops and settles first.
    Route& path();
    // The follower for the move being built, or the next one after path(),
    // a turn or a wait.
    Route& follow(Follow follower);
    Route& line(Point end);
    Route& bezier(Point control_1, Point control_2, Point end);
    // The last segment ends facing this way.
    Route& facing(double heading);
    // The last segment faces along the path.
    Route& tangent();
    // Along the last segment (0 to 1), or at the start of a turn or wait.
    Route& at(double fraction, std::function<void()> action);
    Route& turnTo(double heading);
    Route& wait(uint32_t ms);

    const Pose&              startPose() const { return start_; }
    const std::vector<Step>& steps() const { return steps_; }
    Pose endPose() const;

  private:
    Route& addSegment(const Bezier& curve);

    Pose              start_;
    Point             position_;
    double            heading_  = 0;
    bool              new_move_    = false;            // path() called since the last segment
    Follow            next_follow_ = Follow::kDefault; // for the next move started
    std::vector<Step> steps_;
};

} // namespace navigatr
