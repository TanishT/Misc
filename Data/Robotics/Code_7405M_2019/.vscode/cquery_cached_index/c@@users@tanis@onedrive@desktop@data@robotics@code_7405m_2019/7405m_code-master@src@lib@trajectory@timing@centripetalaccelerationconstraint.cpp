//
// Created by alexweiss on 7/9/19.
//

#include "CentripetalAccelerationConstraint.hpp"
#include "../../physics/DifferentialDrive.hpp"
#include <cmath>

namespace trajectory {
  CentripetalAccelerationConstraint::CentripetalAccelerationConstraint(units::QAngularAcceleration max_centripetal_accel) {
    max_centripetal_accel_ = max_centripetal_accel;
  }
  units::QSpeed CentripetalAccelerationConstraint::getMaxVelocity(geometry::Pose2dWithCurvature state) {
    return units::Qabs(units::Qsqrt(max_centripetal_accel_ / state.curvature() / state.curvature()));
  }
  physics::DifferentialDrive::MinMaxAcceleration CentripetalAccelerationConstraint::getMinMaxAcceleration(
      geometry::Pose2dWithCurvature state, units::QSpeed velocity) {
    return physics::DifferentialDrive::MinMaxAcceleration::kNoLimits;
  }
}