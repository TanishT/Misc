//
// Created by alexweiss on 11/10/19.
//

#include "DriveTurnAction.hpp"
#include "../../subsystems/Drive.hpp"

namespace auton {
  namespace actions {
    DriveTurnAction::DriveTurnAction(units::Angle angle) {
      angle_ = angle;
    }
    bool DriveTurnAction::isFinished() {
      return subsystems::Drive::instance->isDoneWithTrajectory();
    }
    void DriveTurnAction::start() {
      subsystems::Drive::instance->setTurn(angle_);
    }
    void DriveTurnAction::update() {
    }
    void DriveTurnAction::done() {
      subsystems::Drive::instance->setOpenLoop(util::DriveSignal(0, 0));
    }
  }
}
