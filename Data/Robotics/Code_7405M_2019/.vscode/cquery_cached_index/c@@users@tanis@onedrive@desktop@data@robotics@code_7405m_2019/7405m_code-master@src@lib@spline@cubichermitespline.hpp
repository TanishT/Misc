//
// Created by alexweiss on 7/3/19.
//

#ifndef INC_7405M_CODE_CUBICHERMITESPLINE_HPP
#define INC_7405M_CODE_CUBICHERMITESPLINE_HPP

#include "Spline.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"


namespace spline {
    class CubicHermiteSpline : public Spline {
    public:
        units::QJerk ax_, ay_;
        units::QAcceleration bx_, by_;
        units::QSpeed cx_, cy_;
        units::QLength dx_, dy_;

        units::QLength x(units::QTime t);
        units::QLength y(units::QTime t);
        units::QSpeed dx(units::QTime t);
        units::QSpeed dy(units::QTime t);
        units::QAcceleration ddx(units::QTime t);
        units::QAcceleration ddy(units::QTime t);
        units::QJerk dddx(units::QTime t);
        units::QJerk dddy(units::QTime t);


        CubicHermiteSpline(geometry::Pose2d p0, geometry::Pose2d p1);
        geometry::Translation2d getPoint(units::QTime t);
        geometry::Rotation2d getHeading(units::QTime t);
        units::QSpeed getVelocity(units::QTime t);
        units::QCurvature getCurvature(units::QTime t);
        units::QDCurvatureDt getDCurvature(units::QTime t);
    };
}


#endif //INC_7405M_CODE_CUBICHERMITESPLINE_HPP
