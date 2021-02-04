//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_WAITACTION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_WAITACTION_HPP_

#include "Action.hpp"
#include "../../../lib/meecan_lib.hpp"

namespace auton {
  namespace actions {
    class WaitAction : public Action{
     private:
      units::QTime start_time_;
      units::QTime duration_;
     public:
      WaitAction(units::QTime timeout);
      bool isFinished();
      void start();
      void update();
      void done();
    };
  }
}
#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_WAITACTION_HPP_
