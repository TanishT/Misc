//
// Created by alexweiss on 7/8/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMINGCONSTRAINT_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMINGCONSTRAINT_HPP_

#include "../../geometry/interfaces/State.hpp"
#include "../../physics/DifferentialDrive.hpp"
#include <type_traits>
#include <cmath>

namespace trajectory {
  template <class T>
  class TimingConstraint {
  static_assert(std::is_base_of<geometry::State<T>, T>::value, "T is not derived from State");
   public:
    virtual units::QSpeed getMaxVelocity(T state) = 0;
    virtual physics::DifferentialDrive::MinMaxAcceleration getMinMaxAcceleration(T state, units::QSpeed velocity) = 0;
  };
}
#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMINGCONSTRAINT_HPP_
