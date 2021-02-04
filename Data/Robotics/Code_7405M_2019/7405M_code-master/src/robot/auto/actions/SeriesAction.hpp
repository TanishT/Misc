//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_SERIESACTION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_SERIESACTION_HPP_

#include <list>
#include <memory>
#include "Action.hpp"

namespace auton {
  namespace actions {
    class SeriesAction : public Action {
     private:
      Action* current_;
      std::list<Action*> actions_;
      bool isDoneWithFinal;
     public:
      SeriesAction(std::list<Action*> actions);
      bool isFinished();
      void start();
      void update();
      void done();
    };
  }
}
#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_SERIESACTION_HPP_
