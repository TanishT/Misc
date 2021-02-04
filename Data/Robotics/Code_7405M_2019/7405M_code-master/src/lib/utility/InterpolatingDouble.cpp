#include "InterpolatingDouble.hpp"

namespace util {
  InterpolatingDouble::InterpolatingDouble(double v) {
    val_ = v;
  }
  InterpolatingDouble InterpolatingDouble::interpolate(InterpolatingDouble other, units::Number x) {
    double dydx = other.val_ - val_;
    double searchY = dydx * x.getValue() + val_;
    return InterpolatingDouble(searchY);
  }
  double InterpolatingDouble::inverseInterpolate(InterpolatingDouble upper, InterpolatingDouble query) {
    double upper_to_lower = upper.val_ - val_;
    if (upper_to_lower <= 0) {
      return 0;
    }
    double query_to_lower = query.val_ - val_;
    if (query_to_lower <= 0) {
      return 0;
    }
    return query_to_lower / upper_to_lower;
  }
  bool InterpolatingDouble::operator==(const InterpolatingDouble& other) {
    return (val_ == other.val_);
  }
  bool InterpolatingDouble::operator<(const InterpolatingDouble& other) {
    return (val_ < other.val_);
  }
  bool InterpolatingDouble::operator<=(const InterpolatingDouble& other) {
    return (val_ <= other.val_);
  }
  bool InterpolatingDouble::operator>(const InterpolatingDouble& other) {
    return (val_ > other.val_);
  }
  bool InterpolatingDouble::operator>=(const InterpolatingDouble& other) {
    return (val_ >= other.val_);
  }
}
