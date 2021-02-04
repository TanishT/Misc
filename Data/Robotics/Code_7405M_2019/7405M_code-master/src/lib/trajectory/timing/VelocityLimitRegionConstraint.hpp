//
// Created by alexweiss on 7/9/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_VELOCITYLIMITREGIONCONSTRAINT_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_VELOCITYLIMITREGIONCONSTRAINT_HPP_

#include "TimingConstraint.hpp"
#include "../../geometry/interfaces/ITranslation2d.hpp"
#include "../../geometry/Translation2d.hpp"
#include "../../physics/DifferentialDrive.hpp"
#include <type_traits>

namespace trajectory {
  template<class S>
  class VelocityLimitRegionConstraint : public TimingConstraint<S> {
    static_assert(std::is_base_of<geometry::ITranslation2d < S > , S > ::value, "S must derive from ITranslation2d<S>");
   protected:
    geometry::Translation2d min_corner_;
    geometry::Translation2d max_corner_;
    units::QSpeed velocity_limit_;
   public:
    VelocityLimitRegionConstraint(geometry::Translation2d min_corner, geometry::Translation2d max_corner, units::QSpeed velocity_limit)  {
      min_corner_ = min_corner;
      max_corner_ = max_corner;
      velocity_limit_ = velocity_limit;
    }
    units::QSpeed getMaxVelocity(S state) {
      geometry::Translation2d translation = state.translation();
      if (translation.x() <= max_corner_.x() && translation.x() >= min_corner_.x() &&
          translation.y() <= max_corner_.y() && translation.y() >= min_corner_.y()) {
        return velocity_limit_;
      }
      return INFINITY;
    }
    physics::DifferentialDrive::MinMaxAcceleration getMinMaxAcceleration(S state, units::QSpeed velocity)  {
      return physics::DifferentialDrive::MinMaxAcceleration::kNoLimits;
    }

  };


}
#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_VELOCITYLIMITREGIONCONSTRAINT_HPP_
