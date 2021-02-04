//
// Created by alexweiss on 7/4/19.
//

#include "DCMotorTransmission.hpp"
#include "../utility/Utility.hpp"

namespace physics {
  DCMotorTransmission::DCMotorTransmission(units::QAngularSpeed speed_per_volt, units::QTorque torque_per_volt, units::Number friction_voltage)  :
      speed_per_volt_(speed_per_volt),
      torque_per_volt_(torque_per_volt),
      friction_voltage_(friction_voltage) {

  }
  units::QAngularSpeed DCMotorTransmission::speed_per_volt() {
    return speed_per_volt_;
  }
  units::QTorque DCMotorTransmission::torque_per_volt() {
    return torque_per_volt_;
  }
  units::Number DCMotorTransmission::friction_voltage() {
    return friction_voltage_;
  }
  units::QAngularSpeed DCMotorTransmission::free_speed_at_voltage(units::Number voltage) {
    if (voltage > 0.0 * units::num && voltage - friction_voltage() > 0.0 * units::num) {
      return (voltage - friction_voltage()) * speed_per_volt();
    } else if (voltage < 0.0 * units::num && voltage + friction_voltage() < 0. * units::num) {
      return (voltage + friction_voltage()) * speed_per_volt();
    } else {
      return 0.0;
    }
  }
  units::QTorque DCMotorTransmission::get_torque_at_voltage(units::QAngularSpeed speed, units::Number voltage) {
    units::Number effective_voltage = voltage;
    if (speed > 0*units::rps) {
      effective_voltage -= friction_voltage(); // Forward motion, rolling friction.
    } else if (speed < -0*units::rps) {
      effective_voltage += friction_voltage(); // Reverse motion, rolling friction.
    } else if (voltage > 0.0 * units::num) {
      effective_voltage = MAX(0.0 * units::num, voltage - friction_voltage()); // System is static, forward torque.
    } else if (voltage < -0.0 * units::num) {
      effective_voltage = MIN(0.0 * units::num, voltage + friction_voltage()); // System is static, reverse torque.
    } else {
      return 0.0; // System is idle.
    }
    return torque_per_volt() * (-1 * speed / speed_per_volt() + effective_voltage);
  }
  units::Number DCMotorTransmission::get_voltage_for_torque(units::QAngularSpeed speed, units::QTorque torque) {
    units::Number friction_voltage_offset;
    if (speed > 0*units::rps) {
      friction_voltage_offset = friction_voltage(); // Forward motion, rolling friction.
    } else if (speed < -0*units::rps) {
      friction_voltage_offset = -1 * friction_voltage(); // Reverse motion, rolling friction.
    } else if (torque > 0*units::Nm) {
      friction_voltage_offset = friction_voltage(); // System is static, forward torque.
    } else if (torque < -0*units::Nm) {
      friction_voltage_offset = -1 * friction_voltage(); // System is static, reverse torque.
    } else {
      return 0.0; // System is idle.
    }
    return (torque / torque_per_volt() + speed / speed_per_volt()).getValue() + friction_voltage_offset;
  }
}