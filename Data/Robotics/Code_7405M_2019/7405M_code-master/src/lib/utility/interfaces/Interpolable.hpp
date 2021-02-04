#ifndef WLIB_UTIL_INTERPOLABLE_H
#define WLIB_UTIL_INTERPOLABLE_H

#include <string>
#include "../Units.hpp"

namespace util {
  template<typename T> class Interpolable {
    public:
      virtual T interpolate(const T other, units::Number scale) = 0;
  };
}

#endif
