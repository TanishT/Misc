//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_PATHS_DRIVEMOTIONPLANNER_HPP_
#define INC_7405M_CODE_SRC_ROBOT_PATHS_DRIVEMOTIONPLANNER_HPP_

#include "../../lib/meecan_lib.hpp"
#include <vector>


namespace path_planning {
  class DriveMotionPlanner {
   public:
    static physics::DCMotorTransmission transmission;
    static physics::DifferentialDrive drive_model;
   public:
    static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> generateTrajectory(
        bool reversed,
        std::vector<geometry::Pose2d> waypoints,
        std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> constraints,
        units::QSpeed max_vel,  // inches/s
        units::QAcceleration max_accel,  // inches/s^2
        units::Number max_voltage);

    static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> generateTrajectory(
        bool reversed,
        std::vector<geometry::Pose2d> waypoints,
        std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> constraints,
        units::QSpeed start_vel,
        units::QSpeed end_vel,
        units::QSpeed max_vel,  // inches/s
        units::QAcceleration max_accel,  // inches/s^2
        units::Number max_voltage) ;
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_PATHS_DRIVEMOTIONPLANNER_HPP_
