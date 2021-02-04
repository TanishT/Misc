//
// Created by alexweiss on 8/14/19.
//

#include "DriveMotionPlanner.hpp"
#include "../Constants.hpp"
#include "PathConstants.hpp"

namespace path_planning {
  physics::DCMotorTransmission DriveMotionPlanner::transmission(
      constants::RobotConstants::kDriveSpeedPerVolt,
      constants::RobotConstants::kDriveTorquePerVolt,
      constants::RobotConstants::kDriveFrictionVoltage);
  physics::DifferentialDrive DriveMotionPlanner::drive_model(
      constants::RobotConstants::kRobotMass,
      constants::RobotConstants::kRobotMoment,
      constants::RobotConstants::kRobotAngularDrag,
      constants::RobotConstants::kDriveWheelRadius,
      constants::RobotConstants::kDriveWheelTrackWidth / 2.0 * constants::RobotConstants::kTrackScrubFactor,
      transmission, transmission
  );

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> DriveMotionPlanner::generateTrajectory(
      bool reversed,
      std::vector<geometry::Pose2d> waypoints,
      std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> constraints,
      units::QSpeed max_vel,  // inches/s
      units::QAcceleration max_accel,  // inches/s^2
      units::Number max_voltage) {
    return generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> DriveMotionPlanner::generateTrajectory(
      bool reversed,
      std::vector<geometry::Pose2d> waypoints,
      std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> constraints,
      units::QSpeed start_vel,
      units::QSpeed end_vel,
      units::QSpeed max_vel,  // inches/s
      units::QAcceleration max_accel,  // inches/s^2
      units::Number max_voltage)  {

    trajectory::Trajectory<geometry::Pose2dWithCurvature>
        traj = trajectory::TrajectoryUtil::trajectoryFromSplineWaypoints(waypoints,
                                                                         0.2,
                                                                         0.05,
                                                                         geometry::Rotation2d::fromDegrees(3.0).getRadians());


    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature> *> constraints_list;
    trajectory::DifferentialDriveDynamicsConstraint<geometry::Pose2dWithCurvature> drive_constraints(&drive_model, max_voltage);
    trajectory::CentripetalAccelerationConstraint centripetal_acceleration_constraint(constants::PathConstants::kMaxCentripetalAccel);
    //constraints_list.push_back(&drive_constraints);
    constraints_list.push_back(&centripetal_acceleration_constraint);
    if(constraints.size()) {
      for (trajectory::TimingConstraint<geometry::Pose2dWithCurvature> *c : constraints) {
        constraints_list.push_back(c);
      }
    }

    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>>
        timed_trajectory = trajectory::TimingUtil::timeParameterizeTrajectory(
        false, trajectory::DistanceView<geometry::Pose2dWithCurvature>(&traj), .025, constraints_list,
        start_vel, end_vel, max_vel, max_accel);

    return timed_trajectory;


  }
}