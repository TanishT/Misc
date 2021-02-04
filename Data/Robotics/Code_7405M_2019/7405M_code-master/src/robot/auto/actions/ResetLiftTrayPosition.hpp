//
// Created by alexweiss on 8/14/19.
//

#include "Action.hpp"
#include "../../../lib/meecan_lib.hpp"
#include "../../subsystems/Tray.hpp"
#include "../../subsystems/Lift.hpp"

namespace auton {
  namespace actions {
    class ResetLiftTrayPosition : public Action {
     private:

     public:
      ResetLiftTrayPosition();
      bool isFinished();
      void start();
      void update();
      void done();
    };
  }
}
