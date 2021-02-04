//
// Created by alexweiss on 8/12/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_LOOPS_LOOP_HPP_
#define INC_7405M_CODE_SRC_ROBOT_LOOPS_LOOP_HPP_

#include <functional>

namespace loops {
  class Loop {
   public:
    std::function<void()> onStart;
    std::function<void()> onLoop;
    std::function<void()> onDisable;
  };
}


#endif //INC_7405M_CODE_SRC_ROBOT_LOOPS_LOOP_HPP_
