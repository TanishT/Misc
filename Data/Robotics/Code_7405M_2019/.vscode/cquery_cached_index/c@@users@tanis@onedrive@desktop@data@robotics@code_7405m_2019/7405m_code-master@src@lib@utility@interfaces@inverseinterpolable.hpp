#ifndef WLIB_UTIL_REVERSEINTERPOLABLE_H
#define WLIB_UTIL_REVERSEINTERPOLABLE_H

#include <string>

namespace util {
  template<typename T> class InverseInterpolable {
    public:
      virtual double inverseInterpolate(T upper, T query) = 0;
  };
}

#endif
