// drive.h
// Stage 4, drive: command -> motors, every tick. Portable.

#pragma once
#include "navigatr/config.h"
#include "navigatr/types.h"

namespace navigatr
{

// in/s at full voltage, from the motor rpm, gear ratio and wheel diameter.
inline double topSpeed(const gatr2::DriveMotors& motors) {
    return motors.motor_rpm / 60.0 * motors.gear_ratio * kPi * motors.wheel_diameter;
}

class Drive {
  public:
    virtual ~Drive() = default;
    virtual const char* name() const = 0;
    virtual bool        begin(const Config&) { return true; }
    virtual void        apply(const ChassisSpeeds& speeds) = 0;
    virtual void        stop() = 0;
    // Full speed along each axis, after begin(). 0: the drive can't move that way.
    virtual ChassisSpeeds top() const { return {}; }
};

} // namespace navigatr
