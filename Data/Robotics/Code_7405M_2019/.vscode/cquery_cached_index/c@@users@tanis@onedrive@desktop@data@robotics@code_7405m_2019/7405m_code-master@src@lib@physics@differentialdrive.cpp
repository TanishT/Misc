//
// Created by alexweiss on 7/4/19.
//

#include "DifferentialDrive.hpp"
#include "../utility/Utility.hpp"
#include "../utility/Units.hpp"
#include <cmath>

namespace physics {
  DifferentialDrive::MinMaxAcceleration::MinMaxAcceleration() {
    // No limits.
    min_acceleration_ = -INFINITY * units::mps2;
    max_acceleration_ = INFINITY * units::mps2;
  }
  DifferentialDrive::MinMaxAcceleration::MinMaxAcceleration(units::QAcceleration min_acceleration, units::QAcceleration max_acceleration) {
      min_acceleration_ = min_acceleration;
      max_acceleration_ = max_acceleration;
    }
  units::QAcceleration DifferentialDrive::MinMaxAcceleration::min_acceleration() {
    return min_acceleration_;
  }

  units::QAcceleration DifferentialDrive::MinMaxAcceleration::max_acceleration() {
    return max_acceleration_;
  }
  bool DifferentialDrive::MinMaxAcceleration::valid() {
    return min_acceleration() <= max_acceleration();
  }
  DifferentialDrive::MinMaxAcceleration DifferentialDrive::MinMaxAcceleration::kNoLimits;
  DifferentialDrive::DifferentialDrive(units::QMass mass, units::QMoment moi, units::QAngularDrag angular_drag,
                                       units::QLength wheel_radius, units::QLength effective_wheelbase_radius,
                                       DCMotorTransmission left_transmission, DCMotorTransmission right_transmission) :
  mass_ (mass),
  moi_ (moi),
  angular_drag_ (angular_drag),
  wheel_radius_ (wheel_radius),
  effective_wheelbase_radius_ (effective_wheelbase_radius),
  left_ (left_transmission),
  right_ (right_transmission) {
  }
  units::QMass DifferentialDrive::mass() {
    return mass_;
  }
  units::QMoment DifferentialDrive::moi() {
    return moi_;
  }
  units::QLength DifferentialDrive::wheel_radius() {
    return wheel_radius_;
  }
  units::QLength DifferentialDrive::effective_wheelbase_radius() {
    return effective_wheelbase_radius_;
  }
  DCMotorTransmission* DifferentialDrive::left_transmission() {
    return &left_;
  }
  DCMotorTransmission* DifferentialDrive::right_transmission() {
    return &right_;
  }

  DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> DifferentialDrive::solveForwardKinematics(DifferentialDrive::WheelState<units::QAngularSpeed> wheel_motion) {
    ChassisState<units::QSpeed,  units::QAngularSpeed> chassis_motion;
    chassis_motion.linear_ = wheel_radius_ * (wheel_motion.right_ + wheel_motion.left_) / 2.0;
    chassis_motion.angular_ = (wheel_radius_ * (wheel_motion.right_ - wheel_motion.left_) / (2.0 *
        effective_wheelbase_radius_)).getValue();
    return chassis_motion;
  }
  DifferentialDrive::ChassisState<units::QAcceleration, units::QAngularAcceleration> DifferentialDrive::solveForwardKinematics(DifferentialDrive::WheelState<units::QAngularAcceleration> wheel_motion) {
    ChassisState<units::QAcceleration,  units::QAngularAcceleration> chassis_motion;
    chassis_motion.linear_ = wheel_radius_ * (wheel_motion.right_ + wheel_motion.left_) / 2.0;
    chassis_motion.angular_ = (wheel_radius_ * (wheel_motion.right_ - wheel_motion.left_) / (2.0 *
        effective_wheelbase_radius_)).getValue();
    return chassis_motion;
  }
  DifferentialDrive::WheelState<units::QAngularSpeed> DifferentialDrive::solveInverseKinematics(DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> chassis_motion) {
    WheelState<units::QAngularSpeed> wheel_motion;
    wheel_motion.left_ = (chassis_motion.linear_ - effective_wheelbase_radius_ * chassis_motion.angular_) /
        wheel_radius_;
    wheel_motion.right_ = (chassis_motion.linear_ + effective_wheelbase_radius_ * chassis_motion.angular_) /
        wheel_radius_;

    return wheel_motion;
  }
  DifferentialDrive::WheelState<units::QAngularAcceleration> DifferentialDrive::solveInverseKinematics(DifferentialDrive::ChassisState<units::QAcceleration, units::QAngularAcceleration> chassis_motion) {
    WheelState<units::QAngularAcceleration> wheel_motion;
    wheel_motion.left_ = (chassis_motion.linear_ - effective_wheelbase_radius_ * chassis_motion.angular_) /
        wheel_radius_;
    wheel_motion.right_ = (chassis_motion.linear_ + effective_wheelbase_radius_ * chassis_motion.angular_) /
        wheel_radius_;
    return wheel_motion;
  }

  DifferentialDrive::DriveDynamics DifferentialDrive::solveForwardDynamics(ChassisState<units::QSpeed, units::QAngularSpeed> chassis_velocity,
                                                                           DifferentialDrive::WheelState<units::Number> voltage) {
    DriveDynamics dynamics;
    dynamics.wheel_velocity = solveInverseKinematics(chassis_velocity);
    dynamics.chassis_velocity = chassis_velocity;
    dynamics.curvature = dynamics.chassis_velocity.angular_ / dynamics.chassis_velocity.linear_;
    if (std::isnan( dynamics.curvature.getValue() ))
      dynamics.curvature = 0.0;
    dynamics.voltage = voltage;
    solveForwardDynamics(&dynamics);
    return dynamics;
  }
  DifferentialDrive::DriveDynamics DifferentialDrive::solveForwardDynamics(DifferentialDrive::WheelState<units::QAngularSpeed> wheel_velocity, DifferentialDrive::WheelState<units::Number> voltage) {
    DriveDynamics dynamics;
    dynamics.wheel_velocity = wheel_velocity;
    dynamics.chassis_velocity = solveForwardKinematics(wheel_velocity);
    dynamics.curvature = dynamics.chassis_velocity.angular_ / dynamics.chassis_velocity.linear_;
    if (std::isnan(dynamics.curvature.getValue()))
      dynamics.curvature = 0.0;
    dynamics.voltage = voltage;
    solveForwardDynamics(&dynamics);
    return dynamics;
  }

  void DifferentialDrive::solveForwardDynamics(DifferentialDrive::DriveDynamics* dynamics) {
    bool left_stationary = FEQUALS(dynamics->wheel_velocity.left_, 0.0*units::rps) && units::Qabs(dynamics->voltage.left_) < left_.friction_voltage();
    bool right_stationary = FEQUALS(dynamics->wheel_velocity.right_, 0.0*units::rps) && units::Qabs(dynamics->voltage.right_) < right_.friction_voltage();
    if (left_stationary && right_stationary) {
      // Neither side breaks static friction, so we remain stationary.
      dynamics->wheel_torque.left_ = dynamics->wheel_torque.right_ = 0.0;
      dynamics->chassis_acceleration.linear_ = 0.0*units::mps2;
      dynamics->chassis_acceleration.angular_ = 0.0*units::rps2;
      dynamics->wheel_acceleration.left_ = dynamics->wheel_acceleration.right_ = 0.0;
      dynamics->dcurvature = 0.0;
      return;
    }

    // Solve for motor torques generated on each side.

    dynamics->wheel_torque.left_ = left_.get_torque_at_voltage(dynamics->wheel_velocity.left_, dynamics->voltage.left_);
    dynamics->wheel_torque.right_ = right_.get_torque_at_voltage(dynamics->wheel_velocity.right_, dynamics->voltage.right_);

    // Add forces and torques about the center of mass.
    dynamics->chassis_acceleration.linear_ = (dynamics->wheel_torque.right_ + dynamics->wheel_torque.left_) / (wheel_radius_ * mass_);
    // (Tr - Tl) / r_w * r_wb - drag * w = I * angular_accel
    dynamics->chassis_acceleration.angular_ = effective_wheelbase_radius_ * (dynamics->wheel_torque.right_ - dynamics->wheel_torque.left_) / (wheel_radius_ * moi_) - dynamics->chassis_velocity.angular_ * angular_drag_ / moi_;

    // Solve for change in curvature from angular_ acceleration.
    // total angular_ accel = linear_accel * curvature + v^2 * dcurvature

    // TODO: SOLVE DCURVATURE - UNITS ARE WEIRD
    dynamics->dcurvature = (dynamics->chassis_acceleration.angular_ - dynamics->chassis_acceleration.linear_ * dynamics->curvature).getValue() / (dynamics->chassis_velocity.linear_ * dynamics->chassis_velocity.linear_).getValue();
    if (std::isnan(dynamics->dcurvature.getValue()))
      dynamics->dcurvature = 0.0;

    // Resolve chassis accelerations to each wheel.
    dynamics->wheel_acceleration.left_ = (dynamics->chassis_acceleration.linear_ - (dynamics->chassis_acceleration.angular_ * effective_wheelbase_radius_)) / wheel_radius_;
    dynamics->wheel_acceleration.right_ = (dynamics->chassis_acceleration.linear_ + dynamics->chassis_acceleration.angular_ * effective_wheelbase_radius_) / wheel_radius_;
  }

  DifferentialDrive::DriveDynamics DifferentialDrive::solveInverseDynamics(DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> chassis_velocity, DifferentialDrive::ChassisState<units::QAcceleration, units::QAngularAcceleration> chassis_acceleration) {
    DriveDynamics dynamics;
    dynamics.chassis_velocity = chassis_velocity;
    dynamics.curvature = dynamics.chassis_velocity.angular_ / dynamics.chassis_velocity.linear_;
    if (std::isnan(dynamics.curvature.getValue()))
      dynamics.curvature = 0.0;
    dynamics.chassis_acceleration = chassis_acceleration;

    // TODO: SOLVE DCURVATURE - UNITS ARE WEIRD
    dynamics.dcurvature = (dynamics.chassis_acceleration.angular_ - dynamics.chassis_acceleration.linear_ * dynamics.curvature).getValue() / (dynamics.chassis_velocity.linear_ * dynamics.chassis_velocity.linear_).getValue();
    if (std::isnan(dynamics.dcurvature.getValue()))
      dynamics.dcurvature = 0.0;
    dynamics.wheel_velocity = solveInverseKinematics(chassis_velocity);
    dynamics.wheel_acceleration = solveInverseKinematics(chassis_acceleration);
    solveInverseDynamics(&dynamics);
    return dynamics;
  }
  DifferentialDrive::DriveDynamics DifferentialDrive::solveInverseDynamics(DifferentialDrive::WheelState<units::QAngularSpeed> wheel_velocity, DifferentialDrive::WheelState<units::QAngularAcceleration> wheel_acceleration) {
    DriveDynamics dynamics;
    dynamics.chassis_velocity = solveForwardKinematics(wheel_velocity);
    dynamics.curvature = dynamics.chassis_velocity.angular_ / dynamics.chassis_velocity.linear_;
    if (std::isnan(dynamics.curvature.getValue()))
      dynamics.curvature = 0.0;
    dynamics.chassis_acceleration = solveForwardKinematics(wheel_acceleration);

    // TODO: SOLVE DCURVATURE - UNITS ARE WEIRD
    dynamics.dcurvature = (dynamics.chassis_acceleration.angular_ - dynamics.chassis_acceleration.linear_ * dynamics.curvature).getValue() / (dynamics.chassis_velocity.linear_ * dynamics.chassis_velocity.linear_).getValue();
    if (std::isnan(dynamics.dcurvature.getValue()))
      dynamics.dcurvature = 0.0;
    dynamics.wheel_velocity = wheel_velocity;
    dynamics.wheel_acceleration = wheel_acceleration;
    solveInverseDynamics(&dynamics);
    return dynamics;
  }
  void DifferentialDrive::solveInverseDynamics(DifferentialDrive::DriveDynamics* dynamics) {

    dynamics->wheel_torque.left_ = (wheel_radius_ / 2.0) * (dynamics->chassis_acceleration.linear_ * mass_ -
        dynamics->chassis_acceleration.angular_ * moi_ / effective_wheelbase_radius_ -
        dynamics->chassis_velocity.angular_ * angular_drag_ / effective_wheelbase_radius_);

    dynamics->wheel_torque.right_ = (wheel_radius_ / 2.0) * (dynamics->chassis_acceleration.linear_ * mass_ +
        dynamics->chassis_acceleration.angular_ * moi_ / effective_wheelbase_radius_ +
        dynamics->chassis_velocity.angular_ * angular_drag_ / effective_wheelbase_radius_);

    // Solve for input voltages.
    dynamics->voltage.left_ = left_.get_voltage_for_torque(dynamics->wheel_velocity.left_, dynamics->wheel_torque.left_);
    dynamics->voltage.right_ = right_.get_voltage_for_torque(dynamics->wheel_velocity.right_, dynamics->wheel_torque.right_);
  }
  units::QSpeed DifferentialDrive::getMaxAbsVelocity(units::QCurvature curvature, units::Number max_abs_voltage) {
    units::QAngularSpeed left_speed_at_max_voltage = left_.free_speed_at_voltage(max_abs_voltage);
    units::QAngularSpeed right_speed_at_max_voltage = right_.free_speed_at_voltage(max_abs_voltage);

    if(FEQUALS(curvature, 0.0 * units::m_inv)) {
      return wheel_radius_ * MIN(left_speed_at_max_voltage, right_speed_at_max_voltage);
    }

    if(std::isinf(curvature.getValue())) {
      // Turn in place.  Return value meaning becomes angular velocity.
      units::QAngularSpeed wheel_speed = MIN(left_speed_at_max_voltage, right_speed_at_max_voltage);
      return units::metre * curvature.sign() * wheel_radius_ * wheel_speed / effective_wheelbase_radius_;
    }

    units::QAngularSpeed right_speed_if_left_max = left_speed_at_max_voltage * (effective_wheelbase_radius_ * curvature + 1.0) / (1.0 - effective_wheelbase_radius_ * curvature);
    if (fabs(right_speed_if_left_max) <= right_speed_at_max_voltage + EPSILON*units::rps) {
      return wheel_radius_ * (left_speed_at_max_voltage + right_speed_if_left_max) / 2.0; // Left max is active constraint.
    }

    units::QAngularSpeed left_speed_if_right_max = right_speed_at_max_voltage * (1.0 - effective_wheelbase_radius_ * curvature) / (1.0 + effective_wheelbase_radius_ * curvature);
    return wheel_radius_ * (right_speed_at_max_voltage + left_speed_if_right_max) / 2.0; // Right at max is active constraint.
  }

  DifferentialDrive::MinMaxAcceleration DifferentialDrive::getMinMaxAcceleration(DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> chassis_velocity, units::QCurvature curvature, units::Number max_abs_voltage) {
    MinMaxAcceleration result;
    WheelState<units::QAngularSpeed> wheel_velocities = solveInverseKinematics(chassis_velocity);
    result.min_acceleration_ = units::mps2 * INFINITY;
    result.max_acceleration_ = units::mps2 * -INFINITY;

    // Math:
    // (Tl + Tr) / r_w = m*a
    // (Tr - Tl) / r_w * r_wb - drag*w = i*(a * k + v^2 * dk)

    // 2 equations, 2 unknowns.
    // Solve for a and (Tl|Tr)

    units::RQuantity<std::ratio<1,1>, std::ratio<1,1>, std::ratio<0,1>, std::ratio<0,1>> linear_term = std::isinf(curvature.getValue()) ? 0.0 : mass_ * effective_wheelbase_radius_;
    units::QMoment angular_term_tmp = moi_;
    if(!std::isinf(curvature.getValue())) {
      angular_term_tmp = angular_term_tmp * curvature.getValue();
    }
    else {

    }

    units::RQuantity<std::ratio<1,1>, std::ratio<1,1>, std::ratio<0,1>, std::ratio<0,1>> angular_term = angular_term_tmp / units::metre;
    units::QTorque drag_torque = chassis_velocity.angular_ * angular_drag_;

    for(bool left : {0, 1}) {
      for(double sign : {-1.0, 1.0}) {

        DCMotorTransmission* fixed_transmission = left ? &left_ : &right_;
        DCMotorTransmission* variable_transmission = left ? &right_ : &left_;
        units::QTorque fixed_torque = fixed_transmission->get_torque_at_voltage(wheel_velocities.get(left), sign * max_abs_voltage);
        units::QTorque variable_torque = 0.0*units::Nm;

        //TODO IMPORTANT: CALCULATE VARIABLE TORQUE -- CURRENTLY WRONG
        if (left) {
          variable_torque = -1 * drag_torque + fixed_torque * (linear_term + angular_term) / (linear_term - angular_term);
        } else {
          variable_torque = drag_torque + fixed_torque * (linear_term - angular_term) / (linear_term + angular_term);
        }

        units::Number variable_voltage = variable_transmission->get_voltage_for_torque(wheel_velocities.get(!left), variable_torque);
        if (units::Qabs(variable_voltage) <= max_abs_voltage + EPSILON*units::num) {
          units::QAcceleration accel = 0.0;
          if (std::isinf(curvature.getValue())) {
            accel = ((left ? -1.0 : 1.0) * (fixed_torque - variable_torque) * effective_wheelbase_radius_
                / (moi_ * wheel_radius_) - drag_torque / moi_).getValue() * units::mps2;
          } else {
            accel = (fixed_torque + variable_torque) / (mass_ * wheel_radius_);
          }
          result.min_acceleration_ = MIN(result.min_acceleration_, accel);
          result.max_acceleration_ = MAX(result.max_acceleration_, accel);
        }

      }
    }

    return result;

  }

}
