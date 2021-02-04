#include "InterpolatingLong.hpp"
#include "Units.hpp"

namespace util {
  InterpolatingLong::InterpolatingLong(long v) {
    val_ = v;
  }
  InterpolatingLong InterpolatingLong::interpolate(InterpolatingLong other, units::Number x) {
    long dydx = other.val_ - val_;
    double searchY = dydx * x.getValue() + val_;
    return InterpolatingLong((long) searchY);
  }
  double InterpolatingLong::inverseInterpolate(InterpolatingLong upper, InterpolatingLong query) {
    long upper_to_lower = upper.val_ - val_;
    if (upper_to_lower <= 0) {
      return 0;
    }
    long query_to_lower = query.val_ - val_;
    if (query_to_lower <= 0) {
      return 0;
    }
    return ((double) query_to_lower / upper_to_lower);
  }
  bool InterpolatingLong::operator<(const InterpolatingLong& other) {
    return (val_ < other.val_);
  }
  bool InterpolatingLong::operator<=(const InterpolatingLong& other) {
    return (val_ <= other.val_);
  }
  bool InterpolatingLong::operator>(const InterpolatingLong& other) {
    return (val_ > other.val_);
  }
  bool InterpolatingLong::operator>=(const InterpolatingLong& other) {
    return (val_ >= other.val_);
  }
}
