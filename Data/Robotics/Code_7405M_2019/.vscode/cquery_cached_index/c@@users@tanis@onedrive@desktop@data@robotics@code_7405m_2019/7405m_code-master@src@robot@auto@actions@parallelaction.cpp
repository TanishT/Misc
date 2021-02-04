//
// Created by alexweiss on 8/14/19.
//

#include "ParallelAction.hpp"

namespace auton {
  namespace actions {
    ParallelAction::ParallelAction(std::list<Action*> actions) {
      actions_ = actions;
    }
    bool ParallelAction::isFinished() {
      for(Action* action : actions_) {
        if(!action->isFinished())
          return false;
      }
      return true;
    }
    void ParallelAction::start() {
      for(Action* action : actions_) {
        action->start();
      }
    }
    void ParallelAction::update() {
      for(Action* action : actions_) {
        action->update();
      }
    }
    void ParallelAction::done() {
      for(Action* action : actions_) {
        action->done();
      }
    }
  }
}