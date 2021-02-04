//
// Created by alexweiss on 7/3/19.
//

#ifndef INC_7405M_CODE_SPLINE_HPP
#define INC_7405M_CODE_SPLINE_HPP

#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"

namespace spline {
    class Spline {
    public:
        virtual geometry::Translation2d getPoint(units::QTime t) = 0;
        virtual geometry::Rotation2d getHeading(units::QTime t) = 0;
        virtual units::QCurvature getCurvature(units::QTime t) = 0;
        virtual units::QDCurvatureDt getDCurvature(units::QTime t) = 0;
        virtual units::QSpeed getVelocity(units::QTime t) = 0;
        geometry::Pose2d getPose2d(units::QTime t);
        geometry::Pose2dWithCurvature getPose2dWithCurvature(units::QTime t);
    };
}

#endif //INC_7405M_CODE_SPLINE_HPP
