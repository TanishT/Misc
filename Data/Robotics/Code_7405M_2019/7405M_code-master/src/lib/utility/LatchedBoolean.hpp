#ifndef WLIB_UTIL_LATCHEDBOOLEAN_H
#define WLIB_UTIL_LATCHEDBOOLEAN_H

namespace util {
  class LatchedBoolean {
    private:
      bool activated_ = false;
    public:
      LatchedBoolean();
      bool value(bool v);

  };
}

#endif
