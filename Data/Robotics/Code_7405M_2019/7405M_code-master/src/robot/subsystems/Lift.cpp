#include "Lift.hpp"
#include "Tray.hpp"
#include "../Constants.hpp"

#include <stdio.h>

namespace subsystems {
  Lift::LiftManager Lift::instance;

  Lift::Lift() {
    motor = new pros::Motor(constants::RobotConstants::motor_lift, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    pot = new pros::ADIAnalogIn(7);
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    state = ControlState::OPEN_LOOP;
    lastTray = 0;
  }

  void Lift::setOpenLoop(units::Number control) {
    state = ControlState::OPEN_LOOP;
    demand = control;
  }
  void Lift::setPosition(units::Number control) {
    state = ControlState::POSITION_CONTROL;
    demand = control;
    last_error = 0;
    fromMacro = false;
  }
  void Lift::setFromMacro(units::Number control) {
    state = ControlState::POSITION_CONTROL;
    demand = control;
    last_error = 0;
    fromMacro = true;
  }
  units::Number Lift::get_demand() {
    return demand;
  }
  double Lift::getPositionError() {
    return demand.getValue() - get_position();
  }
  double Lift::getMotorVelocity() {
    return motor->get_actual_velocity();
  }
  double Lift::get_position() {
    return pot->get_value();
  }
  ControlState Lift::getState() {
    return state;
  }
  double Lift::getTrayForDemand(double current) {
    printf("LIFT DEMAND %f %f\n", demand.getValue(), current);
    if(demand.getValue() < 0)
      return current;
    else if(demand.getValue() > constants::RobotConstants::LIFT_STAGE[0])
      return constants::RobotConstants::TRAY_LIFT[1];
    else
      return constants::RobotConstants::TRAY_LIFT[0];
  }
  void Lift::runPID() {
    double error = demand.getValue() - pot->get_value();
    if(std::fabs(error) < 50.0 || (pot->get_value() < 250 && demand.getValue() < 300))
      setOpenLoop(0);
    else {
      if(demand.getValue() < 0)
        error += 75;
      motor->move_velocity(error < 0 ? 200 : -200);
    }
  }
  void Lift::updateOutputs() {
    //("LIFT MOTOR STATE %f %d %d\n", motor->get_temperature(), motor->is_over_current(), motor->is_over_temp());
    if(state == ControlState::OPEN_LOOP)
      motor->move_velocity(demand.getValue());
    else if(state == ControlState::POSITION_CONTROL) {
      if(fromMacro) {
        runPID();
      }
      else {
        lastTray = getTrayForDemand(Tray::instance->get_position());
        if(lastTray == constants::RobotConstants::TRAY_LIFT[0]) {
          runPID();
          double lift_error = get_position() - demand.getValue();
          if(lift_error < constants::RobotConstants::liftErrorBeforeTrayStart)
            Tray::instance->setPosition(lastTray);
        }
        else {
          double tray_error = std::fabs(Tray::instance->get_position() - lastTray);
          Tray::instance->setPosition(lastTray);
          if(tray_error < constants::RobotConstants::trayErrorBeforeLiftStart)
            runPID();
        }
      }
    }
  }
  void Lift::stop() {
    setOpenLoop(0);
  }
  void Lift::tare() {
    printf("TARE %f \n", pros::c::motor_get_position(constants::RobotConstants::motor_lift));
    pros::c::motor_tare_position(constants::RobotConstants::motor_lift);
  }
  void Lift::registerEnabledLoops(loops::Looper* enabledLooper) {
    loops::Loop* thisLoop = new loops::Loop();
    thisLoop->onStart = []() {};
    thisLoop->onLoop = []() {
      Lift::instance->updateOutputs();
    };
    thisLoop->onDisable = []() {};
    enabledLooper->add(std::shared_ptr<loops::Loop>(thisLoop));
  }

}
