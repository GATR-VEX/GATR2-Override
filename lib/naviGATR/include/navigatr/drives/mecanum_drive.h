// mecanum_drive.h
// Mecanum drive: wheel speeds to millivolts, scaled down together past max_mv.
// The functions are portable; the class is PROS only.

#pragma once
#include <array>

#include "navigatr/drives/drive.h"

namespace navigatr
{

// mV per in/s: Feedforward::kv, or worked out from the drive when 0.
double wheelKv(const gatr2::DriveMotors& motors, const Feedforward& feedforward);

// mV: front left, front right, back left, back right.
std::array<double, 4> mecanumVoltages(const ChassisSpeeds& speeds, const gatr2::DriveMotors& motors,
                                      const Feedforward& feedforward);

// Full speed forward, sideways and turning, each on its own.
ChassisSpeeds mecanumTop(const gatr2::DriveMotors& motors);

class MecanumDrive : public Drive {
  public:
    const char*   name() const override { return "mecanum"; }
    bool          begin(const Config& config) override; // needs all four corners
    void          apply(const ChassisSpeeds& speeds) override;
    void          stop() override;
    ChassisSpeeds top() const override { return mecanumTop(motors_); }

  private:
    gatr2::DriveMotors motors_;
    Feedforward        feedforward_; // kv filled in
};

} // namespace navigatr
