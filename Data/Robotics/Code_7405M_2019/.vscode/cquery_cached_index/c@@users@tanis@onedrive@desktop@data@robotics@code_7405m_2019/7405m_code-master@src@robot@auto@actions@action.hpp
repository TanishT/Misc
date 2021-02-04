//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_ACTION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_ACTION_HPP_

namespace auton {
  namespace actions {
    class Action {
     public:
      virtual bool isFinished() = 0;
      virtual void start() = 0;
      virtual void update() = 0;
      virtual void done() = 0;
    };
  }
}
#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_ACTION_HPP_
