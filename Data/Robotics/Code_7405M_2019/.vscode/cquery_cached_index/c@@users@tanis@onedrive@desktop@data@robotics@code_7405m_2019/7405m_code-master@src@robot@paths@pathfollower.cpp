//
// Created by alexweiss on 8/15/19.
//

#include "PathFollower.hpp"
#include "../Constants.hpp"
#include "PathConstants.hpp"
#include <cmath>

namespace path_planning {
  Output::Output() {
    Output(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }
  Output::Output(units::QAngularSpeed left_velocity,
                 units::QAngularSpeed right_velocity,
                 units::QAngularAcceleration left_accel,
                 units::QAngularAcceleration right_accel,
                 units::Number left_feedforward_voltage,
                 units::Number right_feedforward_voltage) {
    left_velocity_ = left_velocity;
    right_velocity_ = right_velocity;
    left_accel_ = left_accel;
    right_accel_ = right_accel;
    left_feedforward_voltage_ = left_feedforward_voltage;
    right_feedforward_voltage_ = right_feedforward_voltage;

  }
  void Output::flip() {
    units::QAngularSpeed tmp_left_velocity = left_velocity_;
    left_velocity_ = -1*right_velocity_;
    right_velocity_ = -1*tmp_left_velocity;

    units::QAngularAcceleration tmp_left_accel = left_accel_;
    left_accel_ = -1*right_accel_;
    right_accel_ = -1*tmp_left_accel;

    units::Number tmp_left_feedforward = left_feedforward_voltage_;
    left_feedforward_voltage_ = -1*right_feedforward_voltage_;
    right_feedforward_voltage_ = -1*tmp_left_feedforward;
  }

  PathFollower::PathFollower(trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> traj, FollowerType follower_type) :
  current_trajectory_(traj) {
    physics::DCMotorTransmission transmission(
        constants::RobotConstants::kDriveSpeedPerVolt,
        constants::RobotConstants::kDriveTorquePerVolt,
        constants::RobotConstants::kDriveFrictionVoltage);
    model_ = new physics::DifferentialDrive(
        constants::RobotConstants::kRobotMass,
        constants::RobotConstants::kRobotMoment,
        constants::RobotConstants::kRobotAngularDrag,
        constants::RobotConstants::kDriveWheelRadius,
        constants::RobotConstants::kDriveWheelTrackWidth / 2.0 * constants::RobotConstants::kTrackScrubFactor,
        transmission, transmission
    );
    dt_ = 0;
    last_time_ = INFINITY;
    mFollowerType_ = follower_type;
    is_reversed_ = false;
  }
  bool PathFollower::isDone() {
    return current_trajectory_.isDone();
  }
  geometry::Pose2d PathFollower::error() {
    return error_;
  }
  trajectory::TimedState<geometry::Pose2dWithCurvature> PathFollower::setpoint() {
    return setpoint_;
  }
  Output PathFollower::update(units::QTime timestamp, geometry::Pose2d current_state) {
    if (current_trajectory_.getProgress() == 0.0 && std::isinf(last_time_.getValue())) {
      last_time_ = timestamp;
    }

    dt_ = timestamp - last_time_;
    last_time_ = timestamp;
    trajectory::TrajectorySamplePoint<trajectory::TimedState<geometry::Pose2dWithCurvature>> sample_point = current_trajectory_.advance(dt_.getValue());
    setpoint_ = sample_point.state();
    if(!current_trajectory_.isDone()) {
      units::QSpeed velocity_m = setpoint_.velocity();
      units::QAcceleration acceleration_m = setpoint_.acceleration();
      units::QCurvature curvature_m = setpoint_.state().curvature();
      units::QDCurvatureDs dcurvature_ds_m = setpoint_.state().dcurvature();
      physics::DifferentialDrive::DriveDynamics dynamics = model_->solveInverseDynamics(
          physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(velocity_m, velocity_m * curvature_m),
          physics::DifferentialDrive::ChassisState<units::QAcceleration, units::QAngularAcceleration>(acceleration_m,
                                             acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds_m));
      error_ = current_state.inverse().transformBy(setpoint_.state().pose());

      if (mFollowerType_ == FollowerType::FEEDFORWARD_ONLY) {
        current_output_ = Output(dynamics.wheel_velocity.left_, dynamics.wheel_velocity.right_,
            dynamics.wheel_acceleration.left_, dynamics.wheel_acceleration.right_,
            dynamics.voltage.left_, dynamics.voltage.right_);
      } else if (mFollowerType_ == FollowerType::PURE_PURSUIT) {
        current_output_ = updatePurePursuit(dynamics, current_state);
      } else if (mFollowerType_ == FollowerType::PID) {
        current_output_ = updatePID(dynamics, current_state);
      } else if (mFollowerType_ == FollowerType::NONLINEAR_FEEDBACK) {
        current_output_ = updateNonlinearFeedback(dynamics, current_state);
      }
    }
    else {
      current_output_ = Output();
    }
    return current_output_;

  }
  Output PathFollower::updatePID(physics::DifferentialDrive::DriveDynamics dynamics, geometry::Pose2d current_state) {
    physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> adjusted_velocity;
    // Feedback on longitudinal error (distance).
    units::RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<-1>, std::ratio<0>> kPathKX = 0;
    units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<0>, std::ratio<0>> kPathKY = 0;
    double kPathKTheta = 0;
    adjusted_velocity.linear_ = dynamics.chassis_velocity.linear_ + kPathKX * error_.translation().x();
    adjusted_velocity.angular_ = dynamics.chassis_velocity.angular_ + dynamics.chassis_velocity.linear_ * kPathKY *
            error_.translation().y() + kPathKTheta * error_.rotation().getRadians();
    units::QCurvature curvature = adjusted_velocity.angular_ / adjusted_velocity.linear_;
    if (std::isinf(curvature.getValue())) {
        adjusted_velocity.linear_ = 0.0;
        adjusted_velocity.angular_ = dynamics.chassis_velocity.angular_;
    }
    // Compute adjusted left and right wheel velocities.
    physics::DifferentialDrive::WheelState<units::QAngularSpeed> wheel_velocities = model_->solveInverseKinematics(adjusted_velocity);
    units::Number left_voltage = dynamics.voltage.left_ + (wheel_velocities.left_ - dynamics.wheel_velocity
            .left_) / model_->left_transmission()->speed_per_volt();
    units::Number right_voltage = dynamics.voltage.right_ + (wheel_velocities.right_ - dynamics.wheel_velocity
            .right_) / model_->right_transmission()->speed_per_volt();
    return Output(wheel_velocities.left_, wheel_velocities.right_, dynamics.wheel_acceleration.left_, dynamics
                    .wheel_acceleration.right_, left_voltage, right_voltage);

  }
  Output PathFollower::updatePurePursuit(physics::DifferentialDrive::DriveDynamics dynamics, geometry::Pose2d current_state) {
    double lookahead_time = constants::PathConstants::kPathLookaheadTime.getValue();
    double kLookaheadSearchDt = 0.01;
    trajectory::TimedState<geometry::Pose2dWithCurvature> lookahead_state = current_trajectory_.preview(lookahead_time).state();
    units::QLength actual_lookahead_distance = setpoint_.state().distance(lookahead_state.state());
    while (actual_lookahead_distance < constants::PathConstants::kPathMinLookaheadDistance && current_trajectory_.getRemainingProgress() > lookahead_time) {
      lookahead_time += kLookaheadSearchDt;
      lookahead_state = current_trajectory_.preview(lookahead_time).state();
      actual_lookahead_distance = units::Qabs(setpoint_.state().distance(lookahead_state.state()));
    }
    if (actual_lookahead_distance < constants::PathConstants::kPathMinLookaheadDistance) {
      lookahead_state = trajectory::TimedState<geometry::Pose2dWithCurvature>(geometry::Pose2dWithCurvature(lookahead_state.state()
                                                                 .pose().transformBy(geometry::Pose2d::fromTranslation(geometry::Translation2d(
              (is_reversed_ ? -1.0 : 1.0) * (constants::PathConstants::kPathMinLookaheadDistance -
                  actual_lookahead_distance), 0.0))), 0.0), lookahead_state.t()
          , lookahead_state.velocity(), lookahead_state.acceleration());
    }



    physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> adjusted_velocity;
    adjusted_velocity.linear_ = dynamics.chassis_velocity.linear_ + constants::PathConstants::kPathKX * error_.translation().y();
    // Use pure pursuit to peek ahead along the trajectory and generate a new curvature.
    trajectory::Arc<geometry::Pose2dWithCurvature> arc(current_state, lookahead_state.state());
    units::QCurvature curvature = 1.0 / arc.radius;

    //printf("%f %f\n", adjusted_velocity.linear_, curvature);

    //adjusted_velocity.angular_ = (dynamics.chassis_velocity.angular_);
    adjusted_velocity.angular_ = curvature * dynamics.chassis_velocity.linear_;

    dynamics.chassis_velocity = adjusted_velocity;
    dynamics.wheel_velocity = model_->solveInverseKinematics(adjusted_velocity);
    return Output(dynamics.wheel_velocity.left_, dynamics.wheel_velocity.right_, dynamics.wheel_acceleration.left_, dynamics.wheel_acceleration.right_, dynamics.voltage.left_, dynamics.voltage.right_);
  }

double PathFollower::sinc(double x) {
    if(x < 0.0001) {
        return 1.0;
    }
    return sin(x) / x;
}

Output PathFollower::updateNonlinearFeedback(physics::DifferentialDrive::DriveDynamics dynamics, geometry::Pose2d current_state) {
  printf("\n\n(%f, %f)\n(%f, %f)\n\n", setpoint_.state().translation().x(), setpoint_.state().translation().y(),
          current_state.translation().x(), current_state.translation().y());

    error_ = current_state.inverse().transformBy(setpoint_.state().pose());
    printf("%f\n", error_.translation().norm());

    units::RQuantity<std::ratio<0>, std::ratio<0-2>, std::ratio<0>, std::ratio<0>> kBeta = 2;  // >0.
    units::Number kZeta = 0.7;  // Damping coefficient, [0, 1].

    units::QSpeed targetV = dynamics.chassis_velocity.linear_;
    units::QAngularSpeed targetOmega = dynamics.chassis_velocity.angular_;

    units::QAngularSpeed k1 = 2 * kZeta * units::Qsqrt(targetOmega * targetOmega + kBeta * targetV * targetV);
    units::QAngularSpeed k3 = k1;
    units::RQuantity<std::ratio<0>, std::ratio<0-2>, std::ratio<0>, std::ratio<0>> k2 = kBeta;

    units::QSpeed v = targetV * cos(error_.rotation().getRadians()) +
                      k1 * (cos(current_state.rotation().getRadians()) * error_.translation().x() +
                      sin(current_state.rotation().getRadians()) * error_.translation().y());
    units::QAngularSpeed omega = targetOmega + (k2 * targetV * sinc(error_.rotation().getRadians()) *
                (cos(current_state.rotation().getRadians()) * error_.translation().x() - sin(current_state.rotation().getRadians()) * error_.translation().y()) +
                k3 * error_.rotation().getRadians()).getValue();

    physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> adjusted_velocity;
    adjusted_velocity.linear_ = v;
    adjusted_velocity.angular_ = omega;

    // Compute adjusted left and right wheel velocities.
    dynamics.chassis_velocity = adjusted_velocity;
    dynamics.wheel_velocity = model_->solveInverseKinematics(adjusted_velocity);
    prev_velocity_ = dynamics.chassis_velocity;

    return Output(dynamics.wheel_velocity.left_, dynamics.wheel_velocity.right_, 0.0, 0.0, 0.0, 0.0);
  }
}
