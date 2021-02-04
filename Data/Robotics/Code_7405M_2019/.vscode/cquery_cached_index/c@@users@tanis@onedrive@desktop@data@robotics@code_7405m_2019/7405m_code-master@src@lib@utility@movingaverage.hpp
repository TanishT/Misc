#ifndef WLIB_UTIL_MOVINGAVERAGE_H
#define WLIB_UTIL_MOVINGAVERAGE_H

#include <deque>

namespace util {
  class MovingAverage {
    private:
      int num_values_;
      std::deque<double> values_;
    public:
      MovingAverage(int max_size);
      void AddValue(double new_value);
      double average();
      int getSize() ;
      bool IsUnderMaxSize();
      void clear();

  };
}

#endif
