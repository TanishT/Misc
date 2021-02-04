//
// Created by alexweiss on 10/20/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_MODES_BACKAUTOMODE_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_MODES_BACKAUTOMODE_HPP_

#define BACK_RED false
#define BACK_BLUE true

#include "AutoModeBase.hpp"

namespace auton {
  class BackAutoMode : public AutoModeBase {
    private:
      bool flip_;
    public:
      BackAutoMode(bool flip = false) : flip_(flip) {};
      void routine();
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_MODES_TESTTRAJECTORYMODE_HPP_
