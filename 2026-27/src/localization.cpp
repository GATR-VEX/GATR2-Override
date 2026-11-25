// localization.cpp
// What runs in the localizer.

#include "localization.h"

#include "robot.h"

#include "locagatr/processors/drive_motor_processor.h"
#include "locagatr/processors/imu_processor.h"
#include "locagatr/processors/tracking_wheel_processor.h"
#include "locagatr/processors/wheel_pair_processor.h"
#include "locagatr/sources/control_board_source.h"
#include "locagatr/sources/motor_encoder_source.h"
#include "locagatr/sources/vex_imu_source.h"

namespace
{

// 1. Acquisition.
locagatr::VexImuSource       vex_imu;
locagatr::ControlBoardSource control_board;
locagatr::MotorEncoderSource motor_encoders;

// 2. Processing.
locagatr::TrackingWheelProcessor tracking_wheels;
locagatr::WheelPairProcessor     wheel_pair;
locagatr::ImuProcessor           imu;
locagatr::DriveMotorProcessor    drive;

locagatr::Config config() {
    locagatr::Config c;
    c.rate_hz = 100;
    c.robot   = robot();

    c.sources = {
        {&vex_imu, true},
        {&control_board, true},
        {&motor_encoders, true},
    };
    c.processors = {
        {&tracking_wheels, true},
        {&wheel_pair, false}, // needs a right wheel
        {&imu, true},
        {&drive, true},       // for comparison
    };

    // 3. Estimation: OdometryEstimator.
    c.odometry.forward  = {"left"};
    c.odometry.lateral  = "back";
    c.odometry.yaw      = "IMU";
    c.odometry.attitude = "IMU";

    c.log = true; // /usd/runNNN.csv, a new file each run
    return c;
}

} // namespace

locagatr::Localizer localizer(config());
