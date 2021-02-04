//
// Created by alexweiss on 8/15/19.
//

#include "../Constants.hpp"
#include "Odometry.hpp"
#include <stdio.h>

namespace subsystems {
  Odometry::OdometryManager Odometry::instance;
  Odometry::Odometry() {
    left = new pros::ADIEncoder(5, 6, false);
    right = new pros::ADIEncoder(3, 4, false);
    back = new pros::ADIEncoder(1, 2, false);
  }
  void Odometry::updatePosition() {
    units::QLength leftTravel = (left->get_value()) * 0.0174533 * constants::RobotConstants::kDeadwheelRadius; // .0174533 = PI/180
    units::QLength rightTravel = (right->get_value()) * 0.0174533 * constants::RobotConstants::kDeadwheelRadius;
    units::QLength backTravel = (back->get_value()) * 0.0174533 * constants::RobotConstants::kDeadwheelRadius;
    //units::QLength backTravel = 0;
    //printf("deadwheels %f %f %f\n", leftTravel, rightTravel, backTravel);

    units::Number dTheta = (rightTravel - leftTravel) / (constants::RobotConstants::kDeadwheelBaseWidth);
    units::QLength dY =  (0.5 * (leftTravel + rightTravel));
    units::QLength dX = 0;// - (constants::RobotConstants::kDeadwheelBackTurningRadius *  dTheta);
    //units::QLength dX = 0;

    geometry::Twist2d delta(dY, dX, dTheta * units::radian);
    geometry::Pose2d change = geometry::Pose2d::exp(delta);

    left->reset();
    right->reset();
    back->reset();
    currentPosition = currentPosition.transformBy(change);
    //printf("X %f %f %f\n", currentPosition.translation().x(), currentPosition.translation().y(), currentPosition.rotation().getRadians());
  }
  void Odometry::resetPosition() {
    currentPosition = geometry::Pose2d();
  }
  void Odometry::setCurrentPosition(units::QLength x, units::QLength y, units::Angle theta) {
    currentPosition = geometry::Pose2d(geometry::Translation2d(x, y), geometry::Rotation2d(theta));
  }
  bool Odometry::shouldUpdate() {
    counter++;
    return counter % 3 == 0;
  }
  geometry::Pose2d Odometry::getPosition() {
    return currentPosition;
  }
  void Odometry::registerEnabledLoops(loops::Looper* enabledLooper) {
    loops::Loop* thisLoop = new loops::Loop();
    thisLoop->onStart = []() {};
    thisLoop->onLoop = [] () { if(Odometry::instance->shouldUpdate()) Odometry::instance->updatePosition(); };
    thisLoop->onDisable = []() {};
    enabledLooper->add(std::shared_ptr<loops::Loop>(thisLoop));
  }
  void Odometry::stop() {

  }
}
