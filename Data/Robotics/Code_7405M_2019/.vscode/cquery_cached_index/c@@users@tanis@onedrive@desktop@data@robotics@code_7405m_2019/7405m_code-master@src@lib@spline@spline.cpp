//
// Created by alexweiss on 7/3/19.
//

#include "Spline.hpp"
namespace spline {
    geometry::Pose2d Spline::getPose2d(units::QTime t) {
        return geometry::Pose2d(getPoint(t), getHeading(t));
    }
    geometry::Pose2dWithCurvature Spline::getPose2dWithCurvature(units::QTime t) {
        return geometry::Pose2dWithCurvature(getPose2d(t), getCurvature(t), getDCurvature(t) / getVelocity(t));
    }
}