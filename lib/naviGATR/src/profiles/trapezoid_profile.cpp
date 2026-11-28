// trapezoid_profile.cpp

#include "navigatr/profiles/trapezoid_profile.h"

#include <algorithm>
#include <cmath>

namespace navigatr
{

std::vector<double> speedLimits(const Path& path, const Limits& limits) {
    const std::vector<PathPoint>& p = path.points;
    const std::size_t             n = p.size();
    std::vector<double>           v(n);
    if (n == 0) {
        return v;
    }
    for (std::size_t i = 0; i < n; ++i) {
        double       limit = limits.max_speed;
        const double k     = std::fabs(p[i].curvature);
        if (k > 1e-9 && limits.max_lateral_accel > 0) {
            limit = std::min(limit, std::sqrt(limits.max_lateral_accel / k));
        }
        const PathPoint& before = p[i == 0 ? 0 : i - 1];
        const PathPoint& after  = p[std::min(i + 1, n - 1)];
        const double     ds     = after.s - before.s;
        const double     dh     = std::fabs(after.heading - before.heading);
        if (ds > 1e-9 && dh > 1e-9 && limits.max_turn_rate > 0) {
            limit = std::min(limit, limits.max_turn_rate * ds / dh);
        }
        v[i] = limit;
    }
    // Decelerate to rest at the end.
    v[n - 1] = 0;
    for (std::size_t i = n - 1; i-- > 0;) {
        const double ds = p[i + 1].s - p[i].s;
        v[i]            = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2 * limits.max_decel * ds));
    }
    return v;
}

bool TrapezoidProfile::begin(const Config& config) {
    limits_ = config.limits;
    return limits_.max_speed > 0 && limits_.max_accel > 0 && limits_.max_decel > 0;
}

Trajectory TrapezoidProfile::build(const Path& path) const {
    const std::vector<PathPoint>& p = path.points;
    const std::size_t             n = p.size();
    Trajectory                    trajectory;
    if (n == 0) {
        return trajectory;
    }

    // Accelerate from rest, within the limits.
    std::vector<double> v = speedLimits(path, limits_);
    v[0]                  = 0;
    for (std::size_t i = 1; i < n; ++i) {
        const double ds = p[i].s - p[i - 1].s;
        v[i]            = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2 * limits_.max_accel * ds));
    }

    // Time.
    trajectory.points.resize(n);
    double t = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (i > 0) {
            const double ds = p[i].s - p[i - 1].s;
            t += ds > 0 ? 2 * ds / std::max(v[i - 1] + v[i], 1e-6) : 0.0;
        }
        TrajectoryPoint& point = trajectory.points[i];
        point.t                = t;
        point.s                = p[i].s;
        point.x                = p[i].x;
        point.y                = p[i].y;
        point.v                = v[i];
        point.tangent          = p[i].tangent;
        point.heading          = p[i].heading;
    }
    for (std::size_t i = 0; i + 1 < n; ++i) {
        const double dt = trajectory.points[i + 1].t - trajectory.points[i].t;
        trajectory.points[i].heading_rate =
            dt > 1e-9 ? (trajectory.points[i + 1].heading - trajectory.points[i].heading) / dt : 0.0;
    }
    return trajectory;
}

} // namespace navigatr
