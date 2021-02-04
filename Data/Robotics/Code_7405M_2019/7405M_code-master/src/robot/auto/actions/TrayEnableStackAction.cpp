#include "TrayEnableStackAction.hpp"

namespace auton {
  namespace actions {
    TrayEnableStackAction::TrayEnableStackAction(double s) {
      s_ = s;
    }
    bool TrayEnableStackAction::isFinished() {
        return subsystems::Tray::instance->doneWithScore();
    }
    void TrayEnableStackAction::start() {
        subsystems::Tray::instance->activateScore(s_);
    }
    void TrayEnableStackAction::update() {

    }
    void TrayEnableStackAction::done() {

    }
  }
}
