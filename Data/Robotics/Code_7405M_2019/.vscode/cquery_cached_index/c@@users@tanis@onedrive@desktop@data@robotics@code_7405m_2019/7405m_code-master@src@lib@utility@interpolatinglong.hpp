#ifndef WLIB_UTIL_INTERPOLABLE_LONG_H
#define WLIB_UTIL_INTERPOLABLE_LONG_H

#include "interfaces/Interpolable.hpp"
#include "interfaces/InverseInterpolable.hpp"

namespace util {
  class InterpolatingLong : public Interpolable<InterpolatingLong>, public InverseInterpolable<InterpolatingLong> {
    private:
      long val_;
    public:
      InterpolatingLong(long v);
      InterpolatingLong interpolate(InterpolatingLong other, units::Number x);
      inline long getVal() const { return val_; }
      double inverseInterpolate(InterpolatingLong upper, InterpolatingLong query);
      bool operator<(const InterpolatingLong& other);
      bool operator<=(const InterpolatingLong& other);
      bool operator>(const InterpolatingLong& other);
      bool operator>=(const InterpolatingLong& other);
  };
  static bool operator<(InterpolatingLong const& a, InterpolatingLong const& b) {
    return a.getVal() < b.getVal();
  }
}

#endif
