//
// Created by alexweiss on 11/10/19.
//

#include "OpenLoopTrayAction.hpp"

namespace auton {
  namespace actions {
    OpenLoopTrayAction::OpenLoopTrayAction(units::Number signal, units::QTime duration) {
      duration_ = duration;
      signal_ = signal.getValue();
    }
    bool OpenLoopTrayAction::isFinished() {
      return start_time_ + duration_ < (pros::millis() * units::millisecond);
    }
    void OpenLoopTrayAction::start() {
      printf("started open loop intake\n");
      start_time_ = pros::millis() * units::millisecond;
      subsystems::Tray::instance->setOpenLoop(signal_);
    }
    void OpenLoopTrayAction::update() {

    }
    void OpenLoopTrayAction::done() {
      printf("finished open loop intake\n");
      subsystems::Tray::instance->setOpenLoop(0.0);
    }
  }
}
