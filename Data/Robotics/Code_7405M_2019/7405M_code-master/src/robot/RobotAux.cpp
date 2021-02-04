//
// Created by alexweiss on 8/12/19.
//

#include "Robot.hpp"

namespace meecan {
  void Robot::setupMainLoop() {
    mainLooper = new loops::Looper(0, "main");
    mainLooper->flag_as_main();
    loops::Loop *test = new loops::Loop();

    test->onStart = [](){instance->lastState = -1;};
    test->onDisable = [](){};

    test->onLoop = [](){
      if(pros::competition::is_disabled()) {
        if(instance->lastState != 0)
          instance->disabledInit();
        instance->disabledLoop();
        instance->lastState = 0;
      }
      else if(pros::competition::is_autonomous()) {
        if(instance->lastState != 1)
          instance->autonomousInit();
        instance->autonomousLoop();
        instance->lastState = 1;
      }
      else {
        if(instance->lastState != 2)
          instance->driverInit();
        instance->driverLoop();
        instance->lastState = 2;
      }
    };

    std::shared_ptr<loops::Loop> ptr(test);
    mainLooper->add(ptr);
  }
}

void initialize() {
  meecan::Robot::instance->robotInit();
}

void disabled() {}
void autonomous() {}
void opcontrol() {}