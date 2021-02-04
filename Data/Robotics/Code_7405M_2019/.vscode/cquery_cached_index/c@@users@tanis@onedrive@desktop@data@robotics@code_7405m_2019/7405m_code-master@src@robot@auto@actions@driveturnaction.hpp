//
// Created by alexweiss on 11/10/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_DRIVETURNEACTION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_DRIVETURNEACTION_HPP_

#include "Action.hpp"
#include "../../../lib/meecan_lib.hpp"
#include "../../subsystems/Tray.hpp"

namespace auton {
  namespace actions {
    class DriveTurnAction : public Action {
      private:
        units::Angle angle_;

      public:
        DriveTurnAction(units::Angle angle);
        bool isFinished();
        void start();
        void update();
        void done();
    };
  }
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_OPENLOOPLIFTEACTION_HPP_
