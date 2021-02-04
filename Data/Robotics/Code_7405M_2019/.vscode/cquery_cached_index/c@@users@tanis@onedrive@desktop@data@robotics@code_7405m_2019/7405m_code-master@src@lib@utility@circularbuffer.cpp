#include "CircularBuffer.hpp"
#include <iostream>
#include <memory>

namespace util {
  template<typename T> CircularBuffer<T>::CircularBuffer(int window_size) {
    num_windows_ = window_size;
    samples_ = std::list<T>();
    sum_ = 0;
  }
  template<typename T> void CircularBuffer<T>::clear() {
    samples_.clear();
  }
  template<typename T> void CircularBuffer<T>::add_value(T val) {
    if(samples_.size() >= num_windows_) {
      samples_.pop_front();
    }
    samples_.push_back(val);
    sum_ += val;
  }
  template<typename T> int CircularBuffer<T>::windows() {
    return num_windows_;
  }
  template<typename T> int CircularBuffer<T>::size() {
    return samples_.size();
  }
  template<typename T> T CircularBuffer<T>::sum() {
    return sum_;
  }
  template<typename T> bool CircularBuffer<T>::is_full() {
    return (samples_.size() == num_windows_);
  }
  template<typename T> std::shared_ptr<std::list<T>> CircularBuffer<T>::get_list() {
    return std::make_shared<std::list<T>>(samples_);
  }
}
