//
// Created by alexweiss on 8/10/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_ROBOT_HPP_
#define INC_7405M_CODE_SRC_ROBOT_ROBOT_HPP_

#include "../lib/meecan_lib.hpp"
#include "../tests/testsInclude.hpp"
#include "loops/Loop.hpp"
#include "loops/Looper.hpp"
#include "main.h"


namespace meecan {

  class Robot {
   private:
    loops::Looper* mainLooper;
    loops::Looper* enabledLooper;
    pros::Controller* controller_;
    int lastState = -1;
    int current_auton = 5;
    int lift_state = 0;

   public:
    Robot();
    void setupMainLoop();
    void robotInit();
    void disabledInit();
    void disabledLoop();
    void autonomousInit();
    void autonomousLoop();
    void driverInit();
    void driverLoop();

    struct RobotManager : public util::Singleton<Robot, RobotManager> {};
    static RobotManager instance;
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_ROBOT_HPP_
