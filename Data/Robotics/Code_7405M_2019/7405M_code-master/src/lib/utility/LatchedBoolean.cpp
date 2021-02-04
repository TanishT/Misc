#include "LatchedBoolean.hpp"

namespace util {
  LatchedBoolean::LatchedBoolean() {
  }
  bool LatchedBoolean::value(bool v) {
    if(v && !activated_) {
      activated_ = true;
      return true;
    }
    return false;
  }
}
