//
// Created by alexweiss on 8/9/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_UTILITY_SINGLETON_HPP_
#define INC_7405M_CODE_SRC_LIB_UTILITY_SINGLETON_HPP_

#include <memory>

namespace util {

  template <typename T, typename CONTEXT>
  class Singleton
  {
   public:
    // Very fast inlined operator accessors
    T* operator->() {return mpInstance;}
    const T* operator->() const {return mpInstance;}
    T& operator*() {return *mpInstance;}
    const T& operator*() const {return *mpInstance;}

   protected:
    Singleton()
    {
      static bool static_init = []()->bool {
        mpInstance = new T();
        return true;
      }();
    }

    Singleton(int)
    {
      static bool static_init = []()->bool {
        mpInstance = CONTEXT::init();
        return true;
      }();
    }

   private:
    static T *mpInstance;
  };

  template <typename T, typename CONTEXT>
  T *Singleton<T, CONTEXT>::mpInstance;
}

#endif //INC_7405M_CODE_SRC_LIB_UTILITY_SINGLETON_HPP_
