#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_MODES_NOTHING_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_MODES_NOTHING_HPP_

#include "AutoModeBase.hpp"

namespace auton {
  class DoNothingMode : public AutoModeBase {
    void routine() {};
  };
}

#endif
