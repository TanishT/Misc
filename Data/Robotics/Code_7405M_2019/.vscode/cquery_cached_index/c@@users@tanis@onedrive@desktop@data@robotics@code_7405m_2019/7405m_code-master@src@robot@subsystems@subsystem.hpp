//
// Created by alexweiss on 8/12/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_SUBSYSTEM_HPP_
#define INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_SUBSYSTEM_HPP_

#include "../loops/Looper.hpp"

namespace subsystems {
  enum ControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
    TURN_FOLLOWING,
    POSITION_CONTROL, // position PID control
    SCORE_TRAY,
    TURN_BACK_WHEEL,
  };
  class Subsystem {
   public:
    virtual void writeToLog() {};
    virtual void initializeSystem() {};
    virtual bool checkSystem() {return true;};
    virtual void outputTelemetry() {};
    virtual void stop() = 0;
    virtual void zeroSensors() {}
    virtual void registerEnabledLoops(loops::Looper* enabledLooper) = 0;

  };
}


#endif //INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_SUBSYSTEM_HPP_
