#ifndef WLIB_UTIL_CIRCULARBUFFER_H
#define WLIB_UTIL_CIRCULARBUFFER_H

#include <iostream>
#include <memory>
#include <list>

namespace util {
  template<typename T> class CircularBuffer {
    private:
      int num_windows_;
      std::list<T> samples_;
      T sum_;
    public:
      CircularBuffer(int window_size);
      void clear();
      void add_value(T val);
      int windows();
      int size();
      T sum();
      bool is_full();
      std::shared_ptr<std::list<T>> get_list();

  };
}

#endif
