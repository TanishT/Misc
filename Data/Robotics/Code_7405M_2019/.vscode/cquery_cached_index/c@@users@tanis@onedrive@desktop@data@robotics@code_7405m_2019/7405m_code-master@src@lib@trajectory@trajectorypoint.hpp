//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYPOINT_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYPOINT_HPP_

#include <type_traits>
#include "../geometry/interfaces/State.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"

namespace trajectory {

  template <class S>
  class TrajectoryPoint {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");
     protected:
      S state_;
      int index_;
     public:
      TrajectoryPoint(S state, int index)  {
        state_ = state;
        index_ = index;
      }
      S state()  {
        return state_;
      }
      int index() {
        return index_;
      }
  };


}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYPOINT_HPP_
