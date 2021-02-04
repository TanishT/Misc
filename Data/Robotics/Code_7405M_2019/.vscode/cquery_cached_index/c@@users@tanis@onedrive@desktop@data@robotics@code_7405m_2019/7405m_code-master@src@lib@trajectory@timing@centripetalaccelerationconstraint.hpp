//
// Created by alexweiss on 7/9/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_CENTRIPETALACCELERATIONCONSTRAINT_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_CENTRIPETALACCELERATIONCONSTRAINT_HPP_

#include "TimingConstraint.hpp"
#include "../../physics/DifferentialDrive.hpp"
#include "../../geometry/Pose2dWithCurvature.hpp"

namespace trajectory {
  class CentripetalAccelerationConstraint : public TimingConstraint<geometry::Pose2dWithCurvature> {
   private:
    units::QAngularAcceleration max_centripetal_accel_;
   public:
    CentripetalAccelerationConstraint(units::QAngularAcceleration max_centripetal_accel);
    units::QSpeed getMaxVelocity(geometry::Pose2dWithCurvature state);
    physics::DifferentialDrive::MinMaxAcceleration getMinMaxAcceleration(geometry::Pose2dWithCurvature state, units::QSpeed velocity);
  };
}
#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_CENTRIPETALACCELERATIONCONSTRAINT_HPP_
