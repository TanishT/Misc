//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_OPENLOOPDRIVEACTION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_OPENLOOPDRIVEACTION_HPP_

#include "Action.hpp"
#include "../../../lib/meecan_lib.hpp"
#include "../../subsystems/Drive.hpp"

namespace auton {
  namespace actions {
    class OpenLoopDriveAction : public Action {
     private:
      units::QTime start_time_;
      units::QTime duration_;
      double left_;
      double right_;

     public:
      OpenLoopDriveAction(util::DriveSignal signal, units::QTime duration);
      bool isFinished();
      void start();
      void update();
      void done();
    };
  }
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_OPENLOOPDRIVEACTION_HPP_
