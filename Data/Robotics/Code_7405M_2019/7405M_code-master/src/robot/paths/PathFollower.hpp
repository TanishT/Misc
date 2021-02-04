//
// Created by alexweiss on 8/15/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_PATHS_PATHFOLLOWER_HPP_
#define INC_7405M_CODE_SRC_ROBOT_PATHS_PATHFOLLOWER_HPP_

#include "../../lib/meecan_lib.hpp"
#include <vector>
#include <string>

namespace path_planning {
  enum FollowerType {
    FEEDFORWARD_ONLY,
    PURE_PURSUIT,
    PID,
    NONLINEAR_FEEDBACK
  };

  class Output {
   public:
    units::QAngularSpeed left_velocity_, right_velocity_;
    units::QAngularAcceleration left_accel_, right_accel_;
    units::Number left_feedforward_voltage_, right_feedforward_voltage_;

    Output();
    Output(units::QAngularSpeed left_velocity, units::QAngularSpeed  right_velocity,
           units::QAngularAcceleration left_accel, units::QAngularAcceleration right_accel,
           units::Number left_feedforward_voltage, units::Number right_feedforward_voltage);

    void flip();

  };

  class PathFollower {
   private:
    FollowerType mFollowerType_ = FollowerType::NONLINEAR_FEEDBACK;
    physics::DifferentialDrive* model_;
    trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> current_trajectory_;
    trajectory::TimedState<geometry::Pose2dWithCurvature> setpoint_;
    physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> prev_velocity_;
    geometry::Pose2d error_;
    Output current_output_;
    units::QTime dt_;
    units::QTime last_time_;
    bool is_reversed_;
    double sinc(double x);
   protected:
    Output updatePID(physics::DifferentialDrive::DriveDynamics dynamics, geometry::Pose2d current_state);
    Output updatePurePursuit(physics::DifferentialDrive::DriveDynamics dynamics, geometry::Pose2d current_state);
    Output updateNonlinearFeedback(physics::DifferentialDrive::DriveDynamics dynamics, geometry::Pose2d current_state);

   public:
    PathFollower(trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> traj, FollowerType follower_type);
    bool isDone();
    geometry::Pose2d error();
    trajectory::TimedState<geometry::Pose2dWithCurvature> setpoint();
    Output update(units::QTime timestamp, geometry::Pose2d current_state);
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_PATHS_PATHFOLLOWER_HPP_
