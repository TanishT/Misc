#ifndef CODE_V1_TWIST2D_HPP
#define CODE_V1_TWIST2D_HPP

#include "interfaces/State.hpp"
#include "../utility/Units.hpp"
#include <string>

/**
 * A movement along an arc at constant curvature and velocity.
 */

namespace geometry {
  class Twist2d {
    public:
      units::QLength dx_, dy_;
      units::Angle dtheta_;
      Twist2d(units::QLength dx, units::QLength dy, units::Angle dtheta);
      Twist2d scaled(units::Number scale);
      units::QLength norm();
      units::QCurvature curvature();
      std::string toString();
  };
}

#endif //CODE_V1_TWIST2D_HPP
