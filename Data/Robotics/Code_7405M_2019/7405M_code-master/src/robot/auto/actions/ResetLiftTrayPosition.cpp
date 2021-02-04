#include "ResetLiftTrayPosition.hpp"

namespace auton {
  namespace actions {
    ResetLiftTrayPosition::ResetLiftTrayPosition() {
    }
    bool ResetLiftTrayPosition::isFinished() {
      return true;
    }
    void ResetLiftTrayPosition::start() {
      subsystems::Tray::instance->tare();
      subsystems::Lift::instance->tare();
      subsystems::Tray::instance->setOpenLoop(0.0);
    }
    void ResetLiftTrayPosition::update() {

    }
    void ResetLiftTrayPosition::done() {

    }
  }
}
