#ifndef WLIB_UTIL_UTILITY
#define WLIB_UTIL_UTILITY

#define EPSILON 0.0001
#define INTERPOLATE(a, b, x) (a + (b-a) * x)
#define LIMIT(value, lower, upper) (value < lower ? lower : ((value > upper) ? upper : value))
#define FEQUALS(a, b) ((a - EPSILON <= b) && (a + EPSILON >= b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) > (b) ? (b) : (a))

namespace util {
  template<typename T>
  int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }
}

#endif
