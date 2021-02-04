//
// Created by alexweiss on 7/8/19.
//

#include "testPhysics.hpp"
#include "../../../lib/utility/PolynomialRegression.hpp"
#include "../../../lib/utility/Units.hpp"
#include <stdio.h>
#include <vector>
#include <cmath>

namespace test {
  double testPhysics::rpm_to_rads_per_sec(double rpm) {
    return rpm * (360 * units::degree).Convert(units::radian) / (1 * units::minute).Convert(units::second);
  }
  double testPhysics::inches_to_meters(double inches) {
    return inches * 2.54 / 100.0;
  }
  double testPhysics::feet_to_meters(double feet) {
    return inches_to_meters(feet * 12.0);
  }
  double testPhysics::meters_to_feet(double m) {
    return m * 100 / 2.54 / 12;
  }
  double testPhysics::degrees_to_radians(double deg) {
    return deg * M_PI / 180.0;
  }
  void testPhysics::unitTester() {
    std::printf("%f\n", rpm_to_rads_per_sec(500));
    std::printf("%f\n", rpm_to_rads_per_sec(200));
    std::printf("%f\n", rpm_to_rads_per_sec(0));
    std::printf("%f\n", rpm_to_rads_per_sec(-500));
  }
  void testPhysics::testDriveCharacterization() {
    double ks = 0.75;
    double kv = 0.2;
    double ka = 0.15;

    std::vector<physics::DriveCharacterization::VelocityDataPoint> velocityData;
    // generate velocity data points
    for (double v = 0; v < 1.0; v += 0.01) {
      velocityData.push_back(physics::DriveCharacterization::VelocityDataPoint(MAX(0.0, (v - ks) / kv), v));
    }

    std::vector<physics::DriveCharacterization::AccelerationDataPoint> accelerationData;
    double v, a;
    v = 0;
    // generate acceleration data points
    for (int i = 0; i < 1000; ++i) {
      a = MAX(0.0, 6.0 - kv * v - ks) / ka;
      v += a * EPSILON;
      accelerationData.push_back(physics::DriveCharacterization::AccelerationDataPoint(v, 6.0, a));
    }

    physics::DriveCharacterization::CharacterizationConstants driveConstants;
    driveConstants = physics::DriveCharacterization::characterizeDrive(velocityData, accelerationData);

    assertEquals(driveConstants.ks, ks, EPSILON);
    assertEquals(driveConstants.kv, kv, EPSILON);
    assertEquals(driveConstants.ka, ka, EPSILON);
  }
  void testPhysics::testDCMotorTransmission() {
    physics::DCMotorTransmission motor(rpm_to_rads_per_sec(100.0), .2, 1.5);
    assertEquals(rpm_to_rads_per_sec(1200.0), motor.free_speed_at_voltage(12.0 + 1.5).getValue());
    assertEquals(rpm_to_rads_per_sec(600.0), motor.free_speed_at_voltage(6.0 + 1.5).getValue());
    assertEquals(rpm_to_rads_per_sec(0.0), motor.free_speed_at_voltage(1.4).getValue());
    assertEquals(rpm_to_rads_per_sec(0.0), motor.free_speed_at_voltage(0.0).getValue());

    assertEquals(rpm_to_rads_per_sec(0.0), motor.free_speed_at_voltage(-1.4).getValue());
    assertEquals(rpm_to_rads_per_sec(-600.0), motor.free_speed_at_voltage(-6.0 - 1.5).getValue());
    assertEquals(rpm_to_rads_per_sec(-1200.0), motor.free_speed_at_voltage(-12.0 - 1.5).getValue());

    assertEquals(.2 * 10.5, motor.get_torque_at_voltage(0.0, 12.0).getValue());
    assertEquals(.2 * 3.5, motor.get_torque_at_voltage(0.0, 5.0).getValue());

    assertEquals(-.2 * 10.5, motor.get_torque_at_voltage(0.0, -12.0).getValue());

    assertEquals(0.0, motor.get_torque_at_voltage(0.0, 0.0).getValue());
    assertEquals(0.0, motor.get_torque_at_voltage(0.0, 1.4).getValue());
    assertEquals(0.0, motor.get_torque_at_voltage(0.0, -1.4).getValue());
    assertEquals(0.0, motor.get_torque_at_voltage(rpm_to_rads_per_sec(1200.0), 13.5).getValue());
    assertEquals(-.2 * 1.5, motor.get_torque_at_voltage(rpm_to_rads_per_sec(1200.0), 12.0).getValue());
    assertEquals(.2 * 1.5, motor.get_torque_at_voltage(rpm_to_rads_per_sec(1200.0), 15.0).getValue());
    assertEquals(0.0, motor.get_voltage_for_torque(0.0, 0.0));
    assertEquals(13.5, motor.get_voltage_for_torque(rpm_to_rads_per_sec(1200.0), 0.0));
    assertEquals(-13.5, motor.get_voltage_for_torque(rpm_to_rads_per_sec(-1200.0), 0.0));
    double voltages[] = {0.0, 1.0, 3.34, 6.4, -2.0, 0.9, -5.6, 12.1, 1.499, 1.501};
    double speeds[] = {0.0, 0.1, -0.5, 130.1, 3000.0, -45.0, 666.666};
    for (double speed : speeds) {
      for (double voltage : voltages) {
        units::QTorque torque = motor.get_torque_at_voltage(rpm_to_rads_per_sec(speed), voltage);
        if (fabs(voltage) <= 1.5 && FEQUALS(speed, 0.0)) {
          assertEquals(0.0, torque.getValue());
        } else {
          assertEquals(voltage, motor.get_voltage_for_torque(rpm_to_rads_per_sec(speed), torque), 1e-9);
        }
      }
    }
  }
  void testPhysics::testDifferentialDrive() {

    physics::DCMotorTransmission transmission(rpm_to_rads_per_sec(65.0), 0.35, 1.0);
    physics::DifferentialDrive drive(70.0, 84.0, 0.0, inches_to_meters(2.0), inches_to_meters(25.5) / 2.0, transmission, transmission);
    // Kinematics
    physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> velocity;
    velocity = drive.solveForwardKinematics(physics::DifferentialDrive::WheelState<units::QAngularSpeed>(0.0, 0.0));
    assertEquals(0.0, velocity.linear_.getValue());
    assertEquals(0.0, velocity.angular_.getValue());


    physics::DifferentialDrive::WheelState<units::QAngularSpeed>  wheels;
    wheels = drive.solveInverseKinematics(velocity);
    assertEquals(0.0, wheels.left_.getValue());
    assertEquals(0.0, wheels.right_.getValue());

    velocity = drive.solveForwardKinematics(physics::DifferentialDrive::WheelState<units::QAngularSpeed>(rpm_to_rads_per_sec(65.0 *
        10.0), rpm_to_rads_per_sec(65.0 * 10.0)));
    assertEquals(11.0, meters_to_feet(velocity.linear_.getValue()), 1.0);
    assertEquals(0.0, velocity.angular_.getValue());
    wheels = drive.solveInverseKinematics(velocity);
    assertEquals(rpm_to_rads_per_sec(65.0 * 10.0), wheels.left_.getValue());
    assertEquals(rpm_to_rads_per_sec(65.0 * 10.0), wheels.right_.getValue());
    velocity = drive.solveForwardKinematics(physics::DifferentialDrive::WheelState<units::QAngularSpeed>(rpm_to_rads_per_sec(65.0 *
        -10.0), rpm_to_rads_per_sec(65.0 * -10.0)));
    assertEquals(-11.0, meters_to_feet(velocity.linear_.getValue()), 1.0);
    assertEquals(0.0, velocity.angular_.getValue());
    wheels = drive.solveInverseKinematics(velocity);
    assertEquals(rpm_to_rads_per_sec(-65.0 * 10.0), wheels.left_.getValue());
    assertEquals(rpm_to_rads_per_sec(-65.0 * 10.0), wheels.right_.getValue());

    velocity = drive.solveForwardKinematics(physics::DifferentialDrive::WheelState<units::QAngularSpeed>(rpm_to_rads_per_sec(-65.0 *
        10.0), rpm_to_rads_per_sec(65.0 * 10.0)));
    assertEquals(0.0, meters_to_feet(velocity.linear_.getValue()), 1.0);
    assertEquals(10.0, velocity.angular_.getValue(), 1.0);
    wheels = drive.solveInverseKinematics(velocity);
    assertEquals(rpm_to_rads_per_sec(-65.0 * 10.0), wheels.left_.getValue());
    assertEquals(rpm_to_rads_per_sec(65.0 * 10.0), wheels.right_.getValue());
    velocity = drive.solveForwardKinematics(physics::DifferentialDrive::WheelState<units::QAngularSpeed>(rpm_to_rads_per_sec(65.0 *
        5.0), rpm_to_rads_per_sec(-65.0 * 5.0)));
    assertEquals(0.0, meters_to_feet(velocity.linear_.getValue()), 1.0);
    assertEquals(-5.0, velocity.angular_.getValue(), 1.0);
    wheels = drive.solveInverseKinematics(velocity);
    assertEquals(rpm_to_rads_per_sec(65.0 * 5.0), wheels.left_.getValue());
    assertEquals(rpm_to_rads_per_sec(-65.0 * 5.0), wheels.right_.getValue());

    // Forward dynamics.
    physics::DifferentialDrive::DriveDynamics dynamics = drive.solveForwardDynamics(
        physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(0.0, 0.0),
        physics::DifferentialDrive::WheelState<units::Number>(0.0, 0.0));
    assertEquals(0.0, dynamics.wheel_torque.left_.getValue());
    assertEquals(0.0, dynamics.wheel_torque.right_.getValue());
    assertEquals(0.0, dynamics.wheel_acceleration.left_.getValue());
    assertEquals(0.0, dynamics.wheel_acceleration.right_.getValue());
    assertEquals(0.0, dynamics.chassis_acceleration.linear_.getValue());
    assertEquals(0.0, dynamics.chassis_acceleration.angular_.getValue());

    dynamics = drive.solveForwardDynamics(
        physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(0.0, 0.0),
        physics::DifferentialDrive::WheelState<units::Number>(12.0, 12.0));
    assertEquals(11.0 * .35, dynamics.wheel_torque.left_.getValue());
    assertEquals(11.0 * .35, dynamics.wheel_torque.right_.getValue());
    assertTrue(0.0 < dynamics.wheel_acceleration.left_.getValue());
    assertTrue(0.0 < dynamics.wheel_acceleration.right_.getValue());
    assertEquals(2.0, dynamics.chassis_acceleration.linear_.getValue(), 1.0);
    assertEquals(0.0, dynamics.chassis_acceleration.angular_.getValue());

    dynamics = drive.solveForwardDynamics(
        physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(0.0, 0.0),
        physics::DifferentialDrive::WheelState<units::Number>(-12.0, -12.0));
    assertEquals(-11.0 * .35, dynamics.wheel_torque.left_.getValue());
    assertEquals(-11.0 * .35, dynamics.wheel_torque.right_.getValue());
    assertTrue(0.0 > dynamics.wheel_acceleration.left_.getValue());
    assertTrue(0.0 > dynamics.wheel_acceleration.right_.getValue());
    assertTrue(0.0 > dynamics.chassis_acceleration.linear_.getValue());
    assertEquals(0.0, dynamics.chassis_acceleration.angular_.getValue());

    dynamics = drive.solveForwardDynamics(
        physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(0.0, 0.0),
        physics::DifferentialDrive::WheelState<units::Number>(-12.0, 12.0));
    assertEquals(-11.0 * .35, dynamics.wheel_torque.left_.getValue());
    assertEquals(11.0 * .35, dynamics.wheel_torque.right_.getValue());
    assertTrue(0.0 > dynamics.wheel_acceleration.left_.getValue());
    assertTrue(0.0 < dynamics.wheel_acceleration.right_.getValue());
    assertEquals(0.0, dynamics.chassis_acceleration.linear_.getValue());
    assertTrue(0.0 < dynamics.chassis_acceleration.angular_.getValue());

    // Inverse dynamics.
    dynamics = drive.solveInverseDynamics(
        physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(0.0, 0.0),
        physics::DifferentialDrive::ChassisState<units::QAcceleration, units::QAngularAcceleration>(0.0, 0.0));
    assertEquals(0.0, dynamics.wheel_torque.left_.getValue());
    assertEquals(0.0, dynamics.wheel_torque.right_.getValue());
    assertEquals(0.0, dynamics.voltage.left_);
    assertEquals(0.0, dynamics.voltage.right_);

    dynamics = drive.solveInverseDynamics(
        physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(feet_to_meters(10.0), 0.0),
        physics::DifferentialDrive::ChassisState<units::QAcceleration, units::QAngularAcceleration>(0.0, 0.0));
    assertEquals(0.0, dynamics.wheel_torque.left_.getValue());
    assertEquals(0.0, dynamics.wheel_torque.right_.getValue());
    assertEquals(9.5, dynamics.voltage.left_, 1.0);
    assertEquals(9.5, dynamics.voltage.right_, 1.0);
    dynamics = drive.solveInverseDynamics(
        physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(feet_to_meters(10.0), 0.0),
        physics::DifferentialDrive::ChassisState<units::QAcceleration, units::QAngularAcceleration>(feet_to_meters(2.0), 0.0));
    assertEquals(1.0, dynamics.wheel_torque.left_.getValue(), 0.5);
    assertEquals(1.0, dynamics.wheel_torque.right_.getValue(), 0.5);
    assertEquals(13.0, dynamics.voltage.left_, 1.0);
    assertEquals(13.0, dynamics.voltage.right_, 1.0);
    dynamics = drive.solveInverseDynamics(
        physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(feet_to_meters(10.0), 0.0),
        physics::DifferentialDrive::ChassisState<units::QAcceleration, units::QAngularAcceleration>(feet_to_meters(-2.0), 0.0));
    assertEquals(-1.0, dynamics.wheel_torque.left_.getValue(), 0.5);
    assertEquals(-1.0, dynamics.wheel_torque.right_.getValue(), 0.5);
    assertEquals(6.5, dynamics.voltage.left_, 1.0);
    assertEquals(6.5, dynamics.voltage.right_, 1.0);

    dynamics = drive.solveInverseDynamics(
        physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(feet_to_meters(10.0), degrees_to_radians(45.0)),
        physics::DifferentialDrive::ChassisState<units::QAcceleration, units::QAngularAcceleration>(feet_to_meters(2.0), degrees_to_radians(9.0)));
    std::printf("%f %f %f \n", 1.0, dynamics.wheel_torque.left_.getValue(), 0.5);
    std::printf("%f %f %f \n", 1.0, dynamics.wheel_torque.right_.getValue(), 0.5);
    std::printf("%f %f %f \n", 11.0, dynamics.voltage.left_, 1.0);
    std::printf("%f %f %f \n", 14.0, dynamics.voltage.right_, 1.0);
    std::printf("%f %f \n", dynamics.wheel_velocity.left_, dynamics.wheel_velocity.right_);
    std::printf("%f %f \n", dynamics.wheel_acceleration.left_, dynamics.wheel_acceleration.right_);

    // Max Speed.
    assertEquals(feet_to_meters(13.0), drive.getMaxAbsVelocity(0.0, 12.0).getValue(), 1.0);
    assertEquals(feet_to_meters(6.0), drive.getMaxAbsVelocity(0.0, 6.0).getValue(), 1.0);
    assertEquals(feet_to_meters(3.0), drive.getMaxAbsVelocity(1.0 / drive.effective_wheelbase_radius(), 6.0).getValue(), 1.0);
    assertEquals(feet_to_meters(3.0), drive.getMaxAbsVelocity(-1.0 / drive.effective_wheelbase_radius(), 6.0).getValue(), 1.0);
    assertEquals(rpm_to_rads_per_sec(50.0), drive.getMaxAbsVelocity(INFINITY, 6.0).getValue(), 1.0);
    assertEquals(rpm_to_rads_per_sec(-50.0), drive.getMaxAbsVelocity(-INFINITY, 6.0).getValue(), 1.0);

    // Max acceleration.
    physics::DifferentialDrive::MinMaxAcceleration min_max_accel;
    min_max_accel = drive.getMinMaxAcceleration(physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> (0.0, 0.0), 0.0, 12.0);
    assertEquals(2.0, min_max_accel.max_acceleration_.getValue(), 1.0);
    assertEquals(-2.0, min_max_accel.min_acceleration_.getValue(), 1.0);
    min_max_accel = drive.getMinMaxAcceleration(physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> (0.0, 0.0), 0.0, 6.0);
    assertEquals(1.0, min_max_accel.max_acceleration_.getValue(), 0.5);
    assertEquals(-1.0, min_max_accel.min_acceleration_.getValue(), 0.5);
    min_max_accel = drive.getMinMaxAcceleration(physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> (feet_to_meters(8.0), 0.0), 0.0, 12.0);
    assertEquals(1.0, min_max_accel.max_acceleration_.getValue(), 1.0);
    assertEquals(-4.0, min_max_accel.min_acceleration_.getValue(), 1.0);
    min_max_accel = drive.getMinMaxAcceleration(physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed> (0.0, 0.0), INFINITY, 6.0);
    assertEquals(1.0, min_max_accel.max_acceleration_.getValue(), 0.5);
    assertEquals(-1.0, min_max_accel.min_acceleration_.getValue(), 0.5);
    std::printf("%f %f \n", min_max_accel.max_acceleration_.getValue(), min_max_accel.min_acceleration_.getValue());

  }
}