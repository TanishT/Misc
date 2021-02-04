//
// Created by alexweiss on 7/4/19.
//

#ifndef INC_7405M_CODE_QUINTICHERMITESPLINE_HPP
#define INC_7405M_CODE_QUINTICHERMITESPLINE_HPP

#include "Spline.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include <vector>

namespace spline {
    typedef struct ControlPoint {
        units::Number ddx;
        units::Number ddy;
    } ControlPoint;

    class QuinticHermiteSpline : public Spline {
    protected:
        double x0_, y0_, x1_, y1_;
        double dx0_, dy0_, dx1_, dy1_;
        double ddx0_, ddy0_, ddx1_, ddy1_;

        units::RQuantity<std::ratio<0>, std::ratio<1>, std::ratio<-5>, std::ratio<0>> ax_, ay_;
        units::RQuantity<std::ratio<0>, std::ratio<1>, std::ratio<-4>, std::ratio<0>> bx_, by_;
        units::QJerk cx_, cy_;
        units::QAcceleration dx_, dy_;
        units::QSpeed ex_, ey_;
        units::QLength fx_, fy_;
        bool path_reversed_;

    private:
        // Tuning Parameters for Numerical Methods and Optimizing
        static constexpr units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> kEpsilon = 1.0/10000;
        static constexpr units::QLength kStepSize = 1.0;
        static constexpr units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> kMinDelta = 0.001;
        static constexpr int kSamples = 100; // FOR INTEGRATION
        static constexpr int kMaxIterations = 100;

        QuinticHermiteSpline(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1,
                             double y0, double y1, double dy0, double dy1, double ddy0, double ddy1);

        void computeCoefficients();
        units::QLength x(units::QTime t);
        units::QSpeed dx(units::QTime t);
        units::QAcceleration ddx(units::QTime t);
        units::QJerk dddx(units::QTime t);

        units::QLength y(units::QTime t);
        units::QSpeed dy(units::QTime t);
        units::QAcceleration ddy(units::QTime t);
        units::QJerk dddy(units::QTime t);
        units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> sumDCurvature2(); // Returns integral of dCurve^2 over the spline

        static void runOptimizationIteration(std::vector<QuinticHermiteSpline> *splines);

        // Returns X-Cord of Vertex
        static units::QLength fitParabola(geometry::Translation2d p1, geometry::Translation2d p2, geometry::Translation2d p3);


    public:
        QuinticHermiteSpline(geometry::Pose2d p0, geometry::Pose2d p1);
        geometry::Pose2d getStartPose();
        geometry::Pose2d getEndPose();
        geometry::Translation2d getPoint(units::QTime t);
        units::QSpeed getVelocity(units::QTime t);
        units::QCurvature getCurvature(units::QTime t);
        units::QDCurvatureDt getDCurvature(units::QTime t);
        units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-2>, std::ratio<0>> dCurvature2(units::QTime t);
        geometry::Rotation2d getHeading(units::QTime t);

        static units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> sumDCurvature2(std::vector<QuinticHermiteSpline> splines);
        static units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> optimizeSpline(std::vector<QuinticHermiteSpline> *splines);




    };
}

#endif //INC_7405M_CODE_QUINTICHERMITESPLINE_HPP
