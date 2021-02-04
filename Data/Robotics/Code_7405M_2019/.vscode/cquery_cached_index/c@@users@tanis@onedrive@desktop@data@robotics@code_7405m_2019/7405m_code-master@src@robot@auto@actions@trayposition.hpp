//
// Created by alexweiss on 8/14/19.
//

#include "Action.hpp"
#include "../../../lib/meecan_lib.hpp"
#include "../../subsystems/Tray.hpp"

namespace auton {
  namespace actions {
    class TrayPosition : public Action {
     private:
      units::QTime start_time_;
      double goal_;
      bool limit_velo_;

     public:
      TrayPosition(double goal);
      TrayPosition(double goal, bool limit_velo);
      bool isFinished();
      void start();
      void update();
      void done();
    };
  }
}
