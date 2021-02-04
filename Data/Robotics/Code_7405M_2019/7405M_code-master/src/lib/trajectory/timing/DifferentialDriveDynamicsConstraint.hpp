//
// Created by alexweiss on 7/9/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_DIFFERENTIALDRIVEDYNAMICSCONSTRAINT_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_DIFFERENTIALDRIVEDYNAMICSCONSTRAINT_HPP_

#include "TimingConstraint.hpp"
#include "../../physics/DifferentialDrive.hpp"
#include "../../geometry/interfaces/IPose2d.hpp"
#include "../../geometry/interfaces/ICurvature.hpp"
#include "../../utility/Units.hpp"

namespace trajectory {

  template<class S>
  class DifferentialDriveDynamicsConstraint : public TimingConstraint<S> {
    static_assert(std::is_base_of<geometry::IPose2d < S > , S > ::value, "S must derive from IPose2d<S>");
    static_assert(std::is_base_of<geometry::ICurvature < S > , S > ::value, "S must derive from ICurvature<S>");

   protected:
    physics::DifferentialDrive* drive_;
    units::Number abs_voltage_limit_;
   public:
    DifferentialDriveDynamicsConstraint(physics::DifferentialDrive* drive, units::Number abs_voltage_limit)  {
      drive_ = drive;
      abs_voltage_limit_ = abs_voltage_limit;
    }
    /* ~DifferentialDriveDynamicsConstraint() {
      delete drive_;
    } */
    units::QSpeed getMaxVelocity(S state) {
      units::QCurvature curvature_in_inverse_meters = state.curvature();
      units::QSpeed max_velo_in_meters = drive_->getMaxAbsVelocity(curvature_in_inverse_meters, abs_voltage_limit_);
      return max_velo_in_meters;
    }
    physics::DifferentialDrive::MinMaxAcceleration getMinMaxAcceleration(S state, units::QSpeed velocity)  {
      units::QSpeed velo_in_meters = velocity;
      units::QCurvature curvature_in_inverse_meters = state.curvature();

      physics::DifferentialDrive::MinMaxAcceleration min_max = drive_->getMinMaxAcceleration(physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(
          velo_in_meters, state.curvature() * velocity), curvature_in_inverse_meters, abs_voltage_limit_.getValue());

      return min_max;
    }
  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_DIFFERENTIALDRIVEDYNAMICSCONSTRAINT_HPP_
