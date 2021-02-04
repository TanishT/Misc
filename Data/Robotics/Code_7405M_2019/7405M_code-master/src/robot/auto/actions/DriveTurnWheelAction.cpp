//
// Created by alexweiss on 11/10/19.
//

#include "DriveTurnWheelAction.hpp"
#include "../../subsystems/Drive.hpp"

namespace auton {
  namespace actions {
    DriveTurnWheelAction::DriveTurnWheelAction(units::Angle angle) {
      angle_ = angle;
    }
    bool DriveTurnWheelAction::isFinished() {
      return subsystems::Drive::instance->isDoneWithTrajectory();
    }
    void DriveTurnWheelAction::start() {
      subsystems::Drive::instance->setTurnWheel(angle_);
    }
    void DriveTurnWheelAction::update() {
    }
    void DriveTurnWheelAction::done() {
      subsystems::Drive::instance->setOpenLoop(util::DriveSignal(0, 0));
    }
  }
}
