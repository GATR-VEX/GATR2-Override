// heading_hold.h
// Driver control: holds the heading while the robot drives or strafes with
// the turn stick centered, so it does not twist. Stick units in and out
// (-127 to 127). Portable.
//
//   navigatr::HeadingHold hold;
//   const double turn = hold.turn(dir, strafe, stick_turn, localizer.getHeading(), pros::millis());

#pragma once
#include <cstdint>

namespace navigatr
{

struct HeadingHoldConfig {
    double   kp        = 2.0;  // stick units per degree off. CHECK on the robot
    double   max_turn  = 50;   // largest correction, stick units
    double   deadband  = 5;    // sticks below this count as centered
    uint32_t settle_ms = 250;  // after the turn stick is let go, before holding
};

class HeadingHold {
  public:
    explicit HeadingHold(const HeadingHoldConfig& config = {}) : config_(config) {}

    // The turn to send. turn: the turn stick, positive turns right
    // (clockwise). heading: deg counterclockwise. Holds only while the robot
    // drives or strafes; the turn stick always wins.
    double turn(double forward, double strafe, double turn, double heading, uint32_t now_ms);

    // Forgets the heading, e.g. after an auton route turned the robot.
    void reset() {
        holding_ = false;
        turned_  = false;
    }

    // Holding, and the heading it holds.
    bool   holding() const { return holding_; }
    double target() const { return target_; }

  private:
    HeadingHoldConfig config_;
    bool              holding_       = false;
    bool              turned_        = false; // the turn stick was used since the last hold
    uint32_t          turn_released_ = 0;     // ms
    double            target_        = 0;
};

} // namespace navigatr
