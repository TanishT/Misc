//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_TRAY_HPP_
#define INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_TRAY_HPP_

#include "Subsystem.hpp"
#include "../../lib/meecan_lib.hpp"
#include "main.h"

namespace subsystems {
  class Tray : Subsystem {
    private:
      pros::Motor* motor;
      pros::ADIAnalogIn* pot;
      units::Number demand;
      ControlState current_state;
      int scoring_state_;
      int count_stop_states_;
      bool limit_velo_;
      double score_multi;

      double getMultiplier();

    public:
      Tray();
      ~Tray();
      void setOpenLoop(units::Number control);
      void setPosition(units::Number control);
      void setPosition(units::Number control, bool limit_velo);
      void tare();
      units::Number get_demand();
      double get_position();
      ControlState getState();
      void runPID(bool);
      void updateOutputs();
      void activateScore(double m = 1);
      bool doneWithScore();
      double getMotorVelocity();
      double getPositionError();
      void stop();
      void registerEnabledLoops(loops::Looper* enabledLooper);

      struct TrayManager : util::Singleton<Tray, TrayManager> {};
      static TrayManager instance;
  };
}

#endif
