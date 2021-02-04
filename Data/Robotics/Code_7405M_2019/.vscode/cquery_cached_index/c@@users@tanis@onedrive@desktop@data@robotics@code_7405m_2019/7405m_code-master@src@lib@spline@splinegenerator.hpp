//
// Created by alexweiss on 7/4/19.
//

#ifndef INC_7405M_CODE_SPLINEGENERATOR_HPP
#define INC_7405M_CODE_SPLINEGENERATOR_HPP

#include "Spline.hpp"
#include "CubicHermiteSpline.hpp"
#include "QuinticHermiteSpline.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"
#include <vector>

namespace spline {
    class SplineGenerator {
    private:
        static constexpr units::QLength kMaxDX = 2.0;
        static constexpr units::QLength kMaxDY = .05;
        static constexpr units::Angle kMaxDTheta = 0.1; // RADIANS!!
        static constexpr int kMinSampleSize = 1;

        template <class T>
        static std::vector<geometry::Pose2dWithCurvature> parameterizeSplinesTemplated(std::vector<T> *splines, units::QLength maxDx,
            units::QLength maxDy, units::Angle maxDTheta);

    public:
        static std::vector<geometry::Pose2dWithCurvature> parameterizeSpline(Spline *s, units::QLength maxDx, units::QLength maxDy,
                                                                             units::Angle maxDTheta, units::QTime t0, units::QTime t1);

        static std::vector<geometry::Pose2dWithCurvature> parameterizeSpline(Spline *s);

        static std::vector<geometry::Pose2dWithCurvature>  parameterizeSpline(Spline *s, units::QLength maxDx, units::QLength maxDy,
                                                                              units::Angle maxDTheta);

        static std::vector<geometry::Pose2dWithCurvature> parameterizeSplines(std::vector<CubicHermiteSpline> *splines);
        static std::vector<geometry::Pose2dWithCurvature> parameterizeSplines(std::vector<CubicHermiteSpline> *splines, units::QLength maxDx,
                                                                              units::QLength maxDy, units::Angle maxDTheta);
        static std::vector<geometry::Pose2dWithCurvature> parameterizeSplines(std::vector<QuinticHermiteSpline> *splines);
        static std::vector<geometry::Pose2dWithCurvature> parameterizeSplines(std::vector<QuinticHermiteSpline> *splines, units::QLength maxDx,
                                                                              units::QLength maxDy, units::Angle maxDTheta);

        static void getSegmentArc(Spline *s, std::vector<geometry::Pose2dWithCurvature> *rv, units::QTime t0, units::QTime t1, units::QLength maxDx,
                                  units::QLength maxDy, units::Angle maxDTheta);

    };

}

#endif //INC_7405M_CODE_SPLINEGENERATOR_HPP
