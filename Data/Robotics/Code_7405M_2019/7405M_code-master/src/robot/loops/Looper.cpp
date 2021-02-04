//
// Created by alexweiss on 8/12/19.
//

#include "Looper.hpp"
#include "../Constants.hpp"
#include <stdexcept>


namespace loops {
    Looper::Looper() : Looper(0, "") {}
    Looper::Looper(int priority_offset) : Looper(priority_offset, "Generic Looper") {}
    Looper::Looper(const char *name) : Looper(0, name) {}
    Looper::Looper(int priority_offset, const char *name) : running_(false),
    is_main_loop_(false),
    control_(),
    runner_(new pros::Task(Looper::loop, (void*)this, TASK_PRIORITY_DEFAULT+priority_offset, TASK_STACK_DEPTH_DEFAULT, name)) {}

    void Looper::add(std::shared_ptr<Loop> loop) {
      control_.take(constants::LooperConstants::mutexDelay);
      if(loop->onStart && loop->onLoop && loop->onDisable) {
        loops_.push_back(loop);
      } else {
        throw std::invalid_argument("ALL METHODS MUST BE INITALIZED");
      }

      control_.give();
    }
    void Looper::enable() {
      if(running_)
        return;
      control_.take(constants::LooperConstants::mutexDelay);
      for(std::shared_ptr<Loop> loop : loops_) {
        loop->onStart();
      }
      running_ = true;
      control_.give();
    }
    void Looper::loop(void* param) {
      Looper* instance = static_cast<Looper*>(param);
      while(1) {
        if(instance->running_) {
          instance->control_.take(constants::LooperConstants::mutexDelay);
          for (std::shared_ptr<Loop> loop : instance->loops_) {
            loop->onLoop();
          }
          instance->control_.give();
        }
        pros::Task::delay(constants::LooperConstants::loopDt);
      }
    }
    void Looper::disable() {
      if(!running_)
        return;
      if(is_main_loop_) {
        throw std::logic_error("YOU MAY NOT DISABLE A MAIN LOOP");
      }
      control_.take(constants::LooperConstants::mutexDelay);
      for(std::shared_ptr<Loop> loop : loops_) {
        loop->onDisable();
      }
      running_ = false;
      control_.give();
    }

    void Looper::flag_as_main() {
      is_main_loop_ = true;
    }
    void Looper::unflag_as_main() {
      is_main_loop_ = false;
    }
}