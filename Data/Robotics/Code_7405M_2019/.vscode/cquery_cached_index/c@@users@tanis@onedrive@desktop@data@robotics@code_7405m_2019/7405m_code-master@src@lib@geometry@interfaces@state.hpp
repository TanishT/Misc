#ifndef WLIB_UTIL_STATE_H
#define WLIB_UTIL_STATE_H

#include "../../utility/Units.hpp"
#include "../../utility/interfaces/CSVLoggable.hpp"
#include "../../utility/interfaces/Interpolable.hpp"

namespace geometry {
  template<typename T> class State : util::CSVLoggable, util::Interpolable<T> {
    public:
      virtual units::QLength distance(T other) = 0;
      virtual bool operator==(T other) = 0;
      virtual std::string toCSV() = 0;
      virtual std::string toString() = 0;
  };
}


#endif
