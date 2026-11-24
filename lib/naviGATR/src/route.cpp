// route.cpp

#include "navigatr/route.h"

#include <cmath>
#include <utility>

namespace navigatr
{

Route::Route(const Pose& start)
    : start_(start), position_{start.x, start.y}, heading_(start.heading) {}

Route& Route::path() {
    new_move_ = true;
    return *this;
}

Route& Route::addSegment(const Bezier& curve) {
    if (steps_.empty() || steps_.back().kind != Step::Kind::kMove || new_move_) {
        steps_.push_back(Step{});
    }
    new_move_ = false;
    Segment segment;
    segment.curve         = curve;
    segment.start_heading = heading_;
    segment.end_heading   = heading_;
    steps_.back().segments.push_back(segment);
    position_ = curve.p1;
    return *this;
}

Route& Route::line(Point end) { return addSegment(Bezier::line(position_, end)); }

Route& Route::bezier(Point control_1, Point control_2, Point end) {
    return addSegment(Bezier{position_, control_1, control_2, end});
}

Route& Route::facing(double heading) {
    if (!steps_.empty() && steps_.back().kind == Step::Kind::kMove) {
        Segment& last    = steps_.back().segments.back();
        last.end_heading = heading;
        last.tangent     = false;
        heading_         = heading;
    }
    return *this;
}

Route& Route::tangent() {
    if (!steps_.empty() && steps_.back().kind == Step::Kind::kMove) {
        Segment& last    = steps_.back().segments.back();
        last.tangent     = true;
        const Point v    = last.curve.velocity(1.0);
        const Point d    = (std::hypot(v.x, v.y) > 1e-9) ? v : last.curve.p1 - last.curve.c2;
        last.end_heading = degrees(std::atan2(d.y, d.x));
        heading_         = last.end_heading;
    }
    return *this;
}

Route& Route::at(double fraction, std::function<void()> action) {
    if (steps_.empty()) {
        return *this;
    }
    Step& step = steps_.back();
    if (step.kind == Step::Kind::kMove) {
        step.segments.back().actions.push_back({fraction, std::move(action)});
    } else {
        step.actions.push_back({0.0, std::move(action)});
    }
    return *this;
}

Route& Route::turnTo(double heading) {
    Step step;
    step.kind     = Step::Kind::kTurn;
    step.position = position_;
    step.heading  = heading;
    steps_.push_back(step);
    heading_ = heading;
    return *this;
}

Route& Route::wait(uint32_t ms) {
    Step step;
    step.kind = Step::Kind::kWait;
    step.ms   = ms;
    steps_.push_back(step);
    return *this;
}

Pose Route::endPose() const {
    Pose p    = start_;
    p.x       = position_.x;
    p.y       = position_.y;
    p.heading = heading_;
    return p;
}

} // namespace navigatr
