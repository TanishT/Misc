//
// Created by alexweiss on 11/10/19.
//

#include "OpenLoopIntakeAction.hpp"

namespace auton {
  namespace actions {
    OpenLoopIntakeAction::OpenLoopIntakeAction(units::Number signal, units::QTime duration) {
      duration_ = duration;
      signal_ = signal.getValue();
    }
    bool OpenLoopIntakeAction::isFinished() {
      return start_time_ + duration_ < (pros::millis() * units::millisecond);
    }
    void OpenLoopIntakeAction::start() {
      printf("started open loop intake\n");
      start_time_ = pros::millis() * units::millisecond;
      subsystems::Intake::instance->setOpenLoop(signal_);
    }
    void OpenLoopIntakeAction::update() {
      if(isFinished() && duration_.getValue() > 0)
        done();
    }
    void OpenLoopIntakeAction::done() {
      printf("finished open loop intake\n");
      subsystems::Intake::instance->setOpenLoop(0.0);
    }
  }
}
