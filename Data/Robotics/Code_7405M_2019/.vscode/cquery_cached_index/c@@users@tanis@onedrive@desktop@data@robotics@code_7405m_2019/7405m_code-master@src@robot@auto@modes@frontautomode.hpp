//
// Created by alexweiss on 10/20/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_MODES_FRONTAUTOMODE_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_MODES_FRONTAUTOMODE_HPP_

#define FRONT_RED true
#define FRONT_BLUE false

#include "AutoModeBase.hpp"

namespace auton {
  class FrontAutoMode : public AutoModeBase {
    private:
      bool flip_;
    public:
      FrontAutoMode(bool flip = false) : flip_(flip) {};
      void routine();
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_MODES_TESTTRAJECTORYMODE_HPP_
