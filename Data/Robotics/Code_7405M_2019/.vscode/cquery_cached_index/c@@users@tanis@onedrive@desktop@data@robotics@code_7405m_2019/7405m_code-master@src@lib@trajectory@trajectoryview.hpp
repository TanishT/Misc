//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYVIEW_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYVIEW_HPP_

#include <type_traits>
#include "../geometry/interfaces/State.hpp"
#include "TrajectorySamplePoint.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"

namespace trajectory {

  template<class S> class Trajectory;

  template <class S>
  class TrajectoryView {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");
   public:
    virtual TrajectorySamplePoint<S> sample(double interpolant) = 0;

    virtual double first_interpolant() = 0;

    virtual double last_interpolant() = 0;

    virtual Trajectory<S>* trajectory() = 0;
  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYVIEW_HPP_
