//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_DRIVE_HPP_
#define INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_DRIVE_HPP_

#include "../paths/PathFollower.hpp"
#include "Subsystem.hpp"
#include "../../lib/meecan_lib.hpp"
#include "main.h"

namespace subsystems {
  class Drive : Subsystem {
   private:
    pros::Motor* frontLeft;
    pros::Motor* frontRight;
    pros::Motor* backLeft;
    pros::Motor* backRight;
    ControlState currentState;
    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> currentTrajectory;
    trajectory::TimedView<geometry::Pose2dWithCurvature>* currentTimedView;
    path_planning::PathFollower* currentFollower;
    units::QTime startTime;
    units::Angle goalAngle;
    units::Angle lastTurnError;
    units::Angle totalTurnError;
    bool forceStopTrajectory_;
    double left_demand = 0, right_demand = 0;
    double left_accel, right_accel;
    double left_feed_forward, right_feed_forward;
    int turnFinishCount;
    double clicksWheel;

   public:
    Drive();
    void stop() ;
    void zeroSensors();
    void setOpenLoop(util::DriveSignal voltage);
    void setFromMacro(util::DriveSignal voltage);
    void setVelocity(util::DriveSignal velocity, util::DriveSignal feedforward);
    void registerEnabledLoops(loops::Looper* enabledLooper);
    void setTrajectory(trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> trajectory);
    void setTurn(units::Angle heading);
    void setTurnWheel(units::Angle heading);
    void updatePathFollower();
    void updateOutputs();
    void setBrakeMode(bool set);
    ControlState getState();
    bool isDoneWithTrajectory();
    void overrideTrajectory();


    struct DriveManager : util::Singleton<Drive, DriveManager> {};
    static DriveManager instance;
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_DRIVE_HPP_
