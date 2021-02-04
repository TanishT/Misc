#ifndef WLIB_UTIL_CSVLOGGABLE_H
#define WLIB_UTIL_CSVLOGGABLE_H

#include <string>

namespace util {
  class CSVLoggable {
    public:
      virtual std::string toCSV() = 0;
  };
}

#endif
