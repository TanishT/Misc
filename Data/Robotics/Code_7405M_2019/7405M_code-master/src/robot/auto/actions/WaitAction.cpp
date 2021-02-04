//
// Created by alexweiss on 8/14/19.
//

#include "WaitAction.hpp"
#include "main.h"


namespace auton {
  namespace actions {
    WaitAction::WaitAction(units::QTime timeout) {
      duration_ = timeout;
    }
    bool WaitAction::isFinished() {
      return start_time_ + duration_ < pros::millis() * units::millisecond;
    }
    void WaitAction::start() {
      start_time_ = pros::millis() * units::millisecond;
    }
    void WaitAction::update() {

    }
    void WaitAction::done() {

    }
  }
}