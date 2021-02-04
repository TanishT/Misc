//
// Created by alexweiss on 7/4/19.
//

#include "SplineGenerator.hpp"
#include "Spline.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"
#include <cmath>
#include <vector>

namespace spline {
    std::vector<geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSpline(Spline *s, units::QLength maxDx, units::QLength maxDy, units::Angle maxDTheta, units::QTime t0, units::QTime t1) {
        std::vector<geometry::Pose2dWithCurvature> rv;
        rv.push_back(s->getPose2dWithCurvature(0.0));
        units::QTime dt = (t1 - t0);
        for (units::QTime t = 0; t < t1; t += dt / kMinSampleSize) {
            getSegmentArc(s, &rv, t, t + dt / kMinSampleSize, maxDx, maxDy, maxDTheta);
        }
        return rv;
    }
    std::vector<geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSpline(Spline *s) {
        return parameterizeSpline(s, kMaxDX, kMaxDY, kMaxDTheta, 0.0, 1.0);
    }
    std::vector<geometry::Pose2dWithCurvature>  SplineGenerator::parameterizeSpline(Spline *s, units::QLength maxDx, units::QLength maxDy, units::Angle maxDTheta) {
        return parameterizeSpline(s, maxDx, maxDy, maxDTheta, 0.0, 1.0);
    }

    template <class T>
    std::vector<geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSplinesTemplated(std::vector<T> *splines, units::QLength maxDx, units::QLength maxDy, units::Angle maxDTheta) {
        static_assert(!std::is_abstract<T>::value, "T must not be abstract");
        static_assert(std::is_base_of<Spline, T>::value, "T must derive from Spline");
        std::vector<geometry::Pose2dWithCurvature> rv;
        if(splines->size() == 0)
            return rv;
        rv.push_back(splines->at(0).getPose2dWithCurvature(0.0));
        for (T s : *splines) {
            std::vector<geometry::Pose2dWithCurvature> samples = parameterizeSpline((Spline*) &s, maxDx, maxDy, maxDTheta);
            samples.erase(samples.begin()); // Startpoint is endpoint of previous spline
            for(geometry::Pose2dWithCurvature sample : samples)
                rv.push_back(sample);
        }
        return rv;
    }

    std::vector<geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSplines(std::vector<CubicHermiteSpline> *splines) {
        return parameterizeSplinesTemplated(splines, kMaxDX, kMaxDY, kMaxDTheta);
    }
    std::vector<geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSplines(std::vector<CubicHermiteSpline> *splines, units::QLength maxDx, units::QLength maxDy, units::Angle maxDTheta) {
        return parameterizeSplinesTemplated(splines, maxDx, maxDy, maxDTheta);
    }
    std::vector<geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSplines(std::vector<QuinticHermiteSpline> *splines) {
        return parameterizeSplinesTemplated(splines, kMaxDX, kMaxDY, kMaxDTheta);
    }
    std::vector<geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSplines(std::vector<QuinticHermiteSpline> *splines, units::QLength maxDx, units::QLength maxDy, units::Angle maxDTheta) {
        return parameterizeSplinesTemplated(splines, maxDx, maxDy, maxDTheta);
    }

    void SplineGenerator::getSegmentArc(Spline *s, std::vector<geometry::Pose2dWithCurvature> *rv, units::QTime t0, units::QTime t1, units::QLength maxDx, units::QLength maxDy, units::Angle maxDTheta) {
        geometry::Translation2d p0 = s->getPoint(t0);
        geometry::Translation2d p1 = s->getPoint(t1);
        geometry::Rotation2d r0 = s->getHeading(t0);
        geometry::Rotation2d r1 = s->getHeading(t1);

        geometry::Pose2d transformation(geometry::Translation2d(p0, p1).rotateBy(r0.inverse()), r1.rotateBy(r0.inverse()));
        geometry::Twist2d twist = geometry::Pose2d::log(transformation);

        if (units::Qabs(twist.dy_) > maxDy || units::Qabs(twist.dx_) > maxDx || units::Qabs(twist.dtheta_) > maxDTheta) {
            getSegmentArc(s, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
            getSegmentArc(s, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
        } else {
            rv->push_back(s->getPose2dWithCurvature(t1));
        }
    }
}