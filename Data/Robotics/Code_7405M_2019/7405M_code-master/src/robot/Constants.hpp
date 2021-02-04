//
// Created by alexweiss on 8/9/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_CONSTANTS_HPP_
#define INC_7405M_CODE_SRC_ROBOT_CONSTANTS_HPP_

#include "../lib/utility/Units.hpp"

namespace constants {
  class RobotConstants {
   public:
    static units::QLength kDriveWheelRadius;
    static units::QMass kRobotMass;
    static units::QMoment kRobotMoment;
    static units::QAngularDrag kRobotAngularDrag;
    static units::QLength kDriveWheelTrackWidth;
    static units::Number kTrackScrubFactor;

    static units::QLength kDeadwheelRadius;
    static units::QLength kDeadwheelBaseWidth;
    static units::QLength kDeadwheelBackTurningRadius;

    static units::QAngularSpeed kDriveSpeedPerVolt; // RADIANS / SECOND / VOTL
    static units::QTorque kDriveTorquePerVolt;
    static units::Number kDriveFrictionVoltage; // voltage to overcome friction (V)

    static double turnKP;
    static double turnKI;
    static double turnKD;

    static int motor_drive_frontleft;
    static int motor_drive_backleft;
    static int motor_drive_frontright;
    static int motor_drive_backright;
    static int motor_intake_left;
    static int motor_intake_right;
    static int motor_tray;
    static int motor_lift;

    static double MAX_TRAY_RPM;
    static double MAX_LIFT_RPM;

    static double LIFT_STAGE[1];
    static double TRAY_LIFT[2];
    static double LIFT_PRESETS[3];
    static double trayErrorBeforeLiftStart;
    static double liftErrorBeforeTrayStart;

    static double TRAY_SCORE;
    static double SCORE_START_INTAKE;
    static double SCORE_END_INTAKE;

    static double BACK_WHEELBASE_RADIUS;
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_CONSTANTS_HPP_
