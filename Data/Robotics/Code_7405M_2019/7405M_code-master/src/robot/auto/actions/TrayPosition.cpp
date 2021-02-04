#include "TrayPosition.hpp"

namespace auton {
  namespace actions {
    TrayPosition::TrayPosition(double goal) {
      goal_ = goal;
      limit_velo_ = false;
    }
    TrayPosition::TrayPosition(double goal, bool limit_velo) {
      goal_ = goal;
      limit_velo_ = limit_velo;
    }
    bool TrayPosition::isFinished() {
      //printf("TP %f %f\n", std::fabs(subsystems::Tray::instance->getMotorVelocity()), std::fabs(subsystems::Tray::instance->getPositionError()));
      return std::fabs(subsystems::Tray::instance->getMotorVelocity()) < 15 && std::fabs(subsystems::Tray::instance->getPositionError()) < 100;
    }
    void TrayPosition::start() {
      subsystems::Tray::instance->setPosition(goal_, limit_velo_);
    }
    void TrayPosition::update() {

    }
    void TrayPosition::done() {
      subsystems::Tray::instance->setOpenLoop(0.0);
    }
  }
}
