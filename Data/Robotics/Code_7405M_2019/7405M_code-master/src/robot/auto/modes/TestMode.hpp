//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_MODES_TESTMODE_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_MODES_TESTMODE_HPP_

#include "AutoModeBase.hpp"
#include "../actions/DriveTrajectory.hpp"
#include "../actions/OpenLoopDriveAction.hpp"
#include "../actions/ParallelAction.hpp"
#include "../actions/SeriesAction.hpp"
#include "../actions/WaitAction.hpp"

namespace auton {
  class TestMode : public AutoModeBase {
    void routine();
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_MODES_TESTMODE_HPP_
