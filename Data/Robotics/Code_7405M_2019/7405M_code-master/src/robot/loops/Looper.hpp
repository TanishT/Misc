//
// Created by alexweiss on 8/12/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_LOOPS_LOOPER_HPP_
#define INC_7405M_CODE_SRC_ROBOT_LOOPS_LOOPER_HPP_

#include <memory>
#include <vector>
#include "Loop.hpp"
#include "LoopConstants.hpp"
#include "../../lib/meecan_lib.hpp"
#include "main.h"

namespace loops {
  class Looper {
   private:
    bool running_;
    bool is_main_loop_;
    std::vector<std::shared_ptr<Loop>> loops_;
    pros::Mutex control_;
    std::shared_ptr<pros::Task> runner_;

   public:
    Looper();
    Looper(int priority_offset);
    Looper(const char* name);
    Looper(int priority_offset, const char* name);
    void add(std::shared_ptr<Loop> loop);
    void enable();
    void disable();
    void flag_as_main();
    void unflag_as_main();
    static void loop(void* param);
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_LOOPS_LOOPER_HPP_
