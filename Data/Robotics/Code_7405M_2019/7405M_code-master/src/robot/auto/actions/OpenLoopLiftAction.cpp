//
// Created by alexweiss on 11/10/19.
//

#include "OpenLoopLiftAction.hpp"

namespace auton {
  namespace actions {
    OpenLoopLiftAction::OpenLoopLiftAction(units::Number signal, units::QTime duration) {
      duration_ = duration;
      signal_ = signal.getValue();
    }
    bool OpenLoopLiftAction::isFinished() {
      return start_time_ + duration_ < (pros::millis() * units::millisecond);
    }
    void OpenLoopLiftAction::start() {
      printf("started open loop intake\n");
      start_time_ = pros::millis() * units::millisecond;
      subsystems::Lift::instance->setOpenLoop(signal_);
    }
    void OpenLoopLiftAction::update() {

    }
    void OpenLoopLiftAction::done() {
      printf("finished open loop intake\n");
      subsystems::Lift::instance->setOpenLoop(0.0);
    }
  }
}
