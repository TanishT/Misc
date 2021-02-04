//
// Created by alexweiss on 8/13/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_UTILITY_INTERPOLATINGMAP_HPP_
#define INC_7405M_CODE_SRC_LIB_UTILITY_INTERPOLATINGMAP_HPP_

#include <map>
#include "InterpolatingDouble.hpp"
#include <stdio.h>
#include <iostream>

namespace util {
  template <typename K, typename V>
  class InterpolatingMap {
   private:
    int max_;
    std::map<K, V> map_;
   public:
    InterpolatingMap() {
      max_ = -1;
    }

    InterpolatingMap(int max_size) {
      max_ = max_size;
    }

   V put(K key, V value) {
      if (max_ > 0 && max_ <= map_.size()) {
        // "Prune" the tree if it is oversize
        map_.erase(map_.begin());
      }
      map_.insert({key, value});
      return value;
    }
    V get(K key) {
      return getInterpolated(key);
    }
    V getInterpolated(K key) {
      // if we have exactly this value in the map, just return it
      if( map_.find(key) != map_.end() ) return map_.at(key);
      // if we are beyond the limits, return the first/last element
      if( key < map_.begin()->first )   return map_.begin()->second;
      if( key > map_.rbegin()->first )  return map_.rbegin()->second;

      auto lower = map_.lower_bound(key) == map_.begin() ? map_.begin() : --( map_.lower_bound(key)) ;
      auto upper = map_.upper_bound(key);

      auto lowerKey = lower->first;
      auto upperKey = upper->first;
      auto lowerVal = lower->second;
      auto upperVal = upper->second;

      return lowerVal.interpolate(upperVal, lowerKey.inverseInterpolate(upperKey, key));
    }
  };
}

#endif //INC_7405M_CODE_SRC_LIB_UTILITY_INTERPOLATINGMAP_HPP_
