//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_IPATHFOLLOWER_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_IPATHFOLLOWER_HPP_

#include "../geometry/Twist2d.hpp"
#include "../geometry/Pose2d.hpp"

namespace trajectory {
    class IPathFollower {
     public:
      virtual geometry::Twist2d steer(geometry::Pose2d current_pose) = 0;
      virtual bool isDone() = 0;
    };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_IPATHFOLLOWER_HPP_
