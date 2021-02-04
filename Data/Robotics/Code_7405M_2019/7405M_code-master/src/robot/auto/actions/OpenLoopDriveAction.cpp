//
// Created by alexweiss on 8/14/19.
//

#include "OpenLoopDriveAction.hpp"

namespace auton {
  namespace actions {
    OpenLoopDriveAction::OpenLoopDriveAction(util::DriveSignal signal, units::QTime duration) {
      duration_ = duration;
      left_ = signal.left();
      right_ = signal.right();
    }
    bool OpenLoopDriveAction::isFinished() {
      return (start_time_.getValue() + duration_.getValue()) * 1000 < (pros::millis());
    }
    void OpenLoopDriveAction::start() {
      printf("started open loop drive\n");
      start_time_ = pros::millis() * units::millisecond;
      subsystems::Drive::instance->setOpenLoop(util::DriveSignal(left_, right_));
    }
    void OpenLoopDriveAction::update() {

    }
    void OpenLoopDriveAction::done() {
      printf("finished open loop drive\n");
      subsystems::Drive::instance->setOpenLoop(util::DriveSignal(0.0, 0.0));
    }
  }
}
