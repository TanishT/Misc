//
// Created by alexweiss on 7/2/19.
//

#include "Twist2d.hpp"
#include "../utility/Utility.hpp"
#include <cmath>
#include <sstream>

namespace geometry {
    Twist2d::Twist2d(units::QLength dx, units::QLength dy, units::Angle dtheta) {
        dx_ = dx;
        dy_ = dy;
        dtheta_ = dtheta;
    }
    Twist2d Twist2d::scaled(units::Number scale) {
        return Twist2d(dx_ * scale, dy_ * scale, dtheta_ * scale);
    }
    units::QLength Twist2d::norm() {
        if (dy_ == 0.0*units::metre)
            return fabs(dx_.getValue());
        return hypot(dx_.getValue(), dy_.getValue());
    }
    units::QCurvature Twist2d::curvature() {
        if (fabs(dtheta_.getValue()) < EPSILON && norm() < EPSILON*units::metre)
            return 0.0;
        return (dtheta_ / norm()) / units::radian;
    }
    std::string Twist2d::toString() {
        std::ostringstream stringStream;
        stringStream << "Twist2d," << std::to_string(dx_.getValue()) << "," << std::to_string(dy_.getValue());
        stringStream << "," << std::to_string(dtheta_.getValue());
        return stringStream.str();
    }
}