#ifndef WLIB_UTIL_DELAYEDBOOLEAN_H
#define WLIB_UTIL_DELAYEDBOOLEAN_H

namespace util {
  class DelayedBoolean {
    private:
      int delay_;
      bool activated = false;
      unsigned int activated_timestamp_;
    public:
      DelayedBoolean(int delay);
      void activate(unsigned int time);
      bool value(unsigned int time);

  };
}

#endif
