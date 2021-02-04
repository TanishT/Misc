#include "DriveSignal.hpp"
#include <string>
#include <sstream>


namespace util {
  DriveSignal::DriveSignal(units::Number l, units::Number r) {
    left_ = l.getValue();
    right_ = r.getValue();
  }
  double DriveSignal::left() {
    return left_;
  }
  double DriveSignal::right() {
    return right_;
  }
  std::string DriveSignal::toCSV() {
    std::ostringstream stringStream;
    stringStream << "DriveSignal," << std::to_string(left_) << "," << std::to_string(right_);
    return stringStream.str();
  }

  DriveSignal DriveSignal::NEUTRAL(0, 0);
}
