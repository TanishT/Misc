//
// Created by alexweiss on 11/10/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_OPENLOOPLIFTEACTION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_OPENLOOPLIFTEACTION_HPP_

#include "Action.hpp"
#include "../../../lib/meecan_lib.hpp"
#include "../../subsystems/Lift.hpp"

namespace auton {
  namespace actions {
    class OpenLoopLiftAction : public Action {
      private:
        units::QTime start_time_;
        units::QTime duration_;
        double signal_;

      public:
        OpenLoopLiftAction(units::Number signal, units::QTime duration);
        bool isFinished();
        void start();
        void update();
        void done();
    };
  }
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_OPENLOOPLIFTEACTION_HPP_
