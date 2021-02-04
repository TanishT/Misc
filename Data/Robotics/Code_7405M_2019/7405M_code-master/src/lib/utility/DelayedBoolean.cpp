#include "DelayedBoolean.hpp"

namespace util {
  DelayedBoolean::DelayedBoolean(int delay) {
    delay_ = delay;
  }
  void DelayedBoolean::activate(unsigned int time) {
    activated = true;
    activated_timestamp_ = time;
  }
  bool DelayedBoolean::value(unsigned int time) {
    if(!activated || time < (activated_timestamp_ + delay_)) {
      return false;
    }
    return true;
  }
}
