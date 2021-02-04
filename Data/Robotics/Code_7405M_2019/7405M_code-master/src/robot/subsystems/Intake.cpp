#include "Intake.hpp"
#include "../Constants.hpp"

#include <stdio.h>

namespace subsystems {
  Intake::IntakeManager Intake::instance;

  Intake::Intake() {
    left = new pros::Motor(constants::RobotConstants::motor_intake_left, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    right = new pros::Motor(constants::RobotConstants::motor_intake_right, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    left->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    state_ = ControlState::OPEN_LOOP;
  }

  void Intake::setOpenLoop(units::Number control) {
    state_ = ControlState::OPEN_LOOP;
    demand = control;
  }
  void Intake::setFromMacro(units::Number control) {
    state_ = ControlState::POSITION_CONTROL;
    demand = control;
  }
  void Intake::updateOutputs() {
    left->move_velocity(demand.getValue());
    right->move_velocity(demand.getValue());
  }
  ControlState Intake::getState() {
    return state_;
  }
  void Intake::stop() {
    setOpenLoop(0);
  }
  void Intake::registerEnabledLoops(loops::Looper* enabledLooper) {
    loops::Loop* thisLoop = new loops::Loop();
    thisLoop->onStart = []() {};
    thisLoop->onLoop = []() {
      Intake::instance->updateOutputs();
    };
    thisLoop->onDisable = []() {};
    enabledLooper->add(std::shared_ptr<loops::Loop>(thisLoop));
  }

}
