//
// Created by alexweiss on 7/3/19.
//

#include "CubicHermiteSpline.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include <cmath>

namespace spline{
    CubicHermiteSpline::CubicHermiteSpline(geometry::Pose2d p0, geometry::Pose2d p1) {
        double x0, x1, dx0, dx1, y0, y1, dy0, dy1;
        double scale = 2 * p0.translation().distance(p1.translation()).getValue();
        x0 = p0.translation().x().getValue();
        x1 = p1.translation().x().getValue();
        dx0 = p0.rotation().cos() * scale;
        dx1 = p1.rotation().cos() * scale;
        y0 = p0.translation().y().getValue();
        y1 = p1.translation().y().getValue();
        dy0 = p0.rotation().sin() * scale;
        dy1 = p1.rotation().sin() * scale;
        ax_ = dx0 + dx1 + 2 * x0 - 2 * x1;
        bx_ = -2 * dx0 - dx1 - 3 * x0 + 3 * x1;
        cx_ = dx0;
        dx_ = x0;
        ay_ = dy0 + dy1 + 2 * y0 - 2 * y1;
        by_ = -2 * dy0 - dy1 - 3 * y0 + 3 * y1;
        cy_ = dy0;
        dy_ = y0;
    }
    geometry::Translation2d CubicHermiteSpline::getPoint(units::QTime t) {
        return geometry::Translation2d(x(t), y(t));
    }
    geometry::Rotation2d CubicHermiteSpline::getHeading(units::QTime t) {
        return geometry::Rotation2d(x(t), y(t));
    }
    units::QSpeed CubicHermiteSpline::getVelocity(units::QTime t) {
        return units::Qsqrt(dx(t) * dx(t) + dy(t) * dy(t));
    }
    units::QCurvature CubicHermiteSpline::getCurvature(units::QTime t) {
        units::RQuantity<std::ratio<0>, std::ratio<2>, std::ratio<-3>, std::ratio<0>> a = (dx(t) * ddy(t) - dy(t) * ddx(t));
        units::RQuantity<std::ratio<0>, std::ratio<3>, std::ratio<-3>, std::ratio<0>> b = (dx(t) * dx(t) + dy(t) * dy(t)) * units::Qsqrt(dx(t) * dx(t) + dy(t) * dy(t));
        return a / b;
    }
    units::QDCurvatureDt CubicHermiteSpline::getDCurvature(units::QTime t) {
      double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t)).getValue();
      double numA = ((dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2).getValue();
      double numB = (3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t))).getValue();
      double num = numA + numB;

      double result = num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
      return result;
    }

    units::QLength CubicHermiteSpline::x(units::QTime t) {
        return t * t * t * ax_ + t * t * bx_ + t * cx_ + dx_;
    }
    units::QLength CubicHermiteSpline::y(units::QTime t) {
        return t * t * t * ay_ + t * t * by_ + t * cy_ + dy_;
    }
    units::QSpeed CubicHermiteSpline::dx(units::QTime t) {
        return 3 * t * t * ax_  + 2 * t * bx_ + cx_;
    }
    units::QSpeed CubicHermiteSpline::dy(units::QTime t) {
        return 3 * t * t * ay_  + 2 * t * by_ + cy_;
    }
    units::QAcceleration CubicHermiteSpline::ddx(units::QTime t) {
        return 6 * t * ax_ + 2 * bx_;
    }
    units::QAcceleration CubicHermiteSpline::ddy(units::QTime t) {
        return 6 * t * ay_ + 2 * by_;
    }
    units::QJerk CubicHermiteSpline::dddx(units::QTime t) {
        return 6 * ax_;
    }
    units::QJerk CubicHermiteSpline::dddy(units::QTime t) {
        return 6 * ay_;
    }
}