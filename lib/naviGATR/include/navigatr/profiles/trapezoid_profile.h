// trapezoid_profile.h
// Accelerate, cruise, decelerate, rest to rest. Slower in tight curves and
// fast heading changes.

#pragma once
#include <vector>

#include "navigatr/profiles/profile.h"

namespace navigatr
{

// in/s per path point: max speed, slower in curves and fast heading changes,
// decelerating to rest at the end. No ramp up from the start.
std::vector<double> speedLimits(const Path& path, const Limits& limits);

class TrapezoidProfile : public Profile {
  public:
    const char* name() const override { return "trapezoid"; }
    bool        begin(const Config& config) override;
    Trajectory  build(const Path& path) const override;

  private:
    Limits limits_;
};

} // namespace navigatr
