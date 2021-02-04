//
// Created by alexweiss on 11/28/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_LIFTPOSITION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_LIFTPOSITION_HPP_

#include "Action.hpp"
#include "../../../lib/meecan_lib.hpp"
#include "../../subsystems/Lift.hpp"

namespace auton {
  namespace actions {
    class LiftPosition : public Action {
      private:
        units::QTime start_time_;
        double goal_;
        bool limit_velo_;

      public:
        LiftPosition(double goal);
        LiftPosition(double goal, bool limit_velo);
        bool isFinished();
        void start();
        void update();
        void done();
    };
  }
}
#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_LIFTPOSITION_HPP_
