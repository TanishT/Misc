//
// Created by alexweiss on 8/14/19.
//

#include "AutoModeRunner.hpp"

namespace auton {
  AutoModeRunner::AutoModeRunner() {
    thread_ = new pros::Task(AutoModeRunner::runAuton, this, TASK_PRIORITY_DEFAULT,
                             TASK_STACK_DEPTH_DEFAULT, "AUTO RUNNER");
    thread_->suspend();
  }
  void AutoModeRunner::setAutoMode(std::shared_ptr<AutoModeBase> new_auto_mode) {
    mode_ = new_auto_mode;
  }
  void AutoModeRunner::start() {
    if(mode_)
      thread_->resume();
  }
  void AutoModeRunner::stop() {
    if(mode_)
      thread_->suspend();
  }
  std::shared_ptr<AutoModeBase> AutoModeRunner::getAutoMode() {
    return mode_;
  }

  void AutoModeRunner::runAuton(void* param) {
    AutoModeRunner* instance = static_cast<AutoModeRunner*>(param);
    instance->mode_->run();
    while(1)
      pros::Task::delay(50);
  };

  AutoModeRunner::AutoModeRunnerManager AutoModeRunner::instance;
}
