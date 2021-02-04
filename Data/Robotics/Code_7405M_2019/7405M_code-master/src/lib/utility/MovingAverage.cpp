#include "MovingAverage.hpp"
#include <deque>

namespace util {
  MovingAverage::MovingAverage(int max_size) {
    num_values_ = max_size;
    values_ = std::deque<double>();
  }
  void MovingAverage::AddValue(double new_value){
    if(values_.size() >= num_values_)
      values_.pop_front();
    values_.push_back(new_value);
  }
  double MovingAverage::average() {
    double total;
    for(int i = 0; i < values_.size(); i++)
      total += values_[i];
    return total / values_.size();
  }
  int MovingAverage::getSize() {
    return values_.size();
  }
  bool MovingAverage::IsUnderMaxSize() {
    return num_values_ > values_.size();
  }
  void MovingAverage::clear() {
    values_.clear();
  }
}
