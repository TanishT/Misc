#include "Displacement1d.hpp"
#include "../utility/Utility.hpp"
#include <cmath>
#include <string>
#include <sstream>

namespace geometry {
  Displacement1d::Displacement1d(units::QLength displacement) {
    displacement_ = displacement;
  }
  units::QLength Displacement1d::distance(Displacement1d other) {
    return units::Qabs((displacement_ - other.displacement_));
  }
  bool Displacement1d::operator==(Displacement1d other) {
    return displacement_ == other.displacement_;
  };
  Displacement1d Displacement1d::interpolate(Displacement1d other, units::Number x) {
    return Displacement1d(INTERPOLATE(displacement_, other.displacement_, x));
  }
  std::string Displacement1d::toCSV() {
    std::ostringstream stringStream;
    stringStream << "Displacement1d,";
    stringStream << std::to_string(displacement_.getValue());
    return stringStream.str();
  }
  std::string Displacement1d::toString() {
    return toCSV();
  }
}
