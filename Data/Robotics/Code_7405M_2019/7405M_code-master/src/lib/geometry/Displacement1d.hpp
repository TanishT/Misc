#ifndef WLIB_UTIL_DISPLACEMENT1D_H
#define WLIB_UTIL_DISPLACEMENT1D_H

#include "interfaces/State.hpp"
#include "../utility/Units.hpp"

namespace geometry {
  class Displacement1d : State<Displacement1d> {
    private:
     units::QLength displacement_;
    public:
      Displacement1d(units::QLength displacement);
      units::QLength distance(Displacement1d other);
      bool operator==(Displacement1d other);
      Displacement1d interpolate(Displacement1d other, units::Number x);
      std::string toCSV();
      std::string toString();
  };
}



#endif
