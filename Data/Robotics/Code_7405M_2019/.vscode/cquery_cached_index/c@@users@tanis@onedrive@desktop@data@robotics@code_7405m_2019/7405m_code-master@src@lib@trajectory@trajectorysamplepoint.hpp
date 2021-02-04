//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYSAMPLEPOINT_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYSAMPLEPOINT_HPP_

#include <type_traits>
#include "../geometry/interfaces/State.hpp"
#include "TrajectoryPoint.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"

namespace trajectory {
  template <class S>
  class TrajectorySamplePoint {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");
    protected:
      S state_;
      int index_floor_;
      int index_ceil_;
    public:
      TrajectorySamplePoint(S state, int index_floor, int index_ceil)  {
        state_ = state;
        index_floor_ = index_floor;
        index_ceil_ = index_ceil;
      }
      TrajectorySamplePoint(TrajectoryPoint<S> point)  {
        state_ = point.state();
        index_floor_ = index_ceil_ = point.index();
      }
      S state() {
        return state_;
      }
      int index_floor()  {
        return index_floor_;
      }
      int index_ceil()  {
        return index_ceil_;
      }
  };



}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYSAMPLEPOINT_HPP_
