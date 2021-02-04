//
// Created by alexweiss on 7/4/19.
//

#include "QuinticHermiteSpline.hpp"
#include <cmath>
#include <stdio.h>

namespace spline {
    QuinticHermiteSpline::QuinticHermiteSpline(geometry::Pose2d p0, geometry::Pose2d p1) {
        double scale = 1.2 * p0.translation().distance(p1.translation()).getValue();
        x0_ = p0.translation().x().getValue();
        x1_ = p1.translation().x().getValue();
        dx0_ = p0.rotation().cos() * scale;
        dx1_ = p1.rotation().cos() * scale;
        ddx0_ = 0;
        ddx1_ = 0;
        y0_ = p0.translation().y().getValue();
        y1_ = p1.translation().y().getValue();
        dy0_ = p0.rotation().sin() * scale;
        dy1_ = p1.rotation().sin() * scale;
        ddy0_ = 0;
        ddy1_ = 0;
        computeCoefficients();
    }
    QuinticHermiteSpline::QuinticHermiteSpline(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1,
                         double y0, double y1, double dy0, double dy1, double ddy0, double ddy1) {

        x0_ = x0;
        x1_ = x1;
        dx0_ = dx0;
        dx1_ = dx1;
        ddx0_ = ddx0;
        ddx1_ = ddx1;

        y0_ = y0;
        y1_ = y1;
        dy0_ = dy0;
        dy1_ = dy1;
        ddy0_ = ddy0;
        ddy1_ = ddy1;

        computeCoefficients();
    }
    void QuinticHermiteSpline::computeCoefficients() {
        ax_ = -6 * x0_ - 3 * dx0_ - 0.5 * ddx0_ + 0.5 * ddx1_ - 3 * dx1_ + 6 * x1_;
        bx_ = 15 * x0_ + 8 * dx0_ + 1.5 * ddx0_ - ddx1_ + 7 * dx1_ - 15 * x1_;
        cx_ = -10 * x0_ - 6 * dx0_ - 1.5 * ddx0_ + 0.5 * ddx1_ - 4 * dx1_ + 10 * x1_;
        dx_ = 0.5 * ddx0_;
        ex_ = dx0_;
        fx_ = x0_;

        ay_ = -6 * y0_ - 3 * dy0_ - 0.5 * ddy0_ + 0.5 * ddy1_ - 3 * dy1_ + 6 * y1_;
        by_ = 15 * y0_ + 8 * dy0_ + 1.5 * ddy0_ - ddy1_ + 7 * dy1_ - 15 * y1_;
        cy_ = -10 * y0_ - 6 * dy0_ - 1.5 * ddy0_ + 0.5 * ddy1_ - 4 * dy1_ + 10 * y1_;
        dy_ = 0.5 * ddy0_;
        ey_ = dy0_;
        fy_ = y0_;
    }
    units::QLength QuinticHermiteSpline::x(units::QTime t) {
        return t * t * t * t * t * ax_ + t * t * t * t * bx_ + t * t * t * cx_ + t * t * dx_ + t * ex_ + fx_;
    }
    units::QSpeed QuinticHermiteSpline::dx(units::QTime t) {
        return 5 * t * t * t * t * ax_ + 4 * t * t * t * bx_ + 3 * t * t * cx_ + 2 * t * dx_ + ex_;
    }
    units::QAcceleration QuinticHermiteSpline::ddx(units::QTime t) {
        return 20 * t * t * t * ax_ + 12 * t * t * bx_ + 6 * t * cx_ + 2 * dx_;
    }
    units::QJerk QuinticHermiteSpline::dddx(units::QTime t) {
        return 60 * t * t * ax_ + 24 * t * bx_ + 6 * cx_;
    }
    units::QLength QuinticHermiteSpline::y(units::QTime t) {
        return t * t * t * t * t * ay_ + t * t * t * t * by_ + t * t * t * cy_ + t * t * dy_ + t* ey_ + fy_;
    }
    units::QSpeed QuinticHermiteSpline::dy(units::QTime t) {
        return 5 * t * t * t * t * ay_ + 4 * t * t * t * by_ + 3 * t * t * cy_ + 2 * t * dy_ + ey_;
    }
    units::QAcceleration QuinticHermiteSpline::ddy(units::QTime t) {
        return 20 * t * t * t * ay_ + 12 * t * t * by_ + 6 * t * cy_ + 2 * dy_;
    }
    units::QJerk QuinticHermiteSpline::dddy(units::QTime t) {
        return 60 * t * t * ay_ + 24 * t * by_ + 6 * cy_;
    }
    geometry::Pose2d QuinticHermiteSpline::getStartPose() {
        return geometry::Pose2d(geometry::Translation2d(x0_, y0_),  geometry::Rotation2d(dx0_, dy0_));
    }
    geometry::Pose2d QuinticHermiteSpline::getEndPose() {
        return geometry::Pose2d(geometry::Translation2d(x1_, y1_),  geometry::Rotation2d(dx1_, dy1_));
    }
    geometry::Translation2d QuinticHermiteSpline::getPoint(units::QTime t) {
        return geometry::Translation2d(x(t), y(t));
    }
    units::QSpeed QuinticHermiteSpline::getVelocity(units::QTime t) {
        return units::Qsqrt(dx(t) * dx(t) + dy(t) * dy(t));
    }
    units::QCurvature QuinticHermiteSpline::getCurvature(units::QTime t) {
      units::RQuantity<std::ratio<0>, std::ratio<2>, std::ratio<-3>, std::ratio<0>> a = (dx(t) * ddy(t) - dy(t) * ddx(t));
      units::RQuantity<std::ratio<0>, std::ratio<3>, std::ratio<-3>, std::ratio<0>> b = (dx(t) * dx(t) + dy(t) * dy(t)) * units::Qsqrt(dx(t) * dx(t) + dy(t) * dy(t));
      return a / b;
    }
    units::QDCurvatureDt QuinticHermiteSpline::getDCurvature(units::QTime t) {
      /* units::RQuantity<std::ratio<0>, std::ratio<2>, std::ratio<-2>, std::ratio<0>>  dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
      units::RQuantity<std::ratio<0>, std::ratio<-5>, std::ratio<5>, std::ratio<0>> a = 1 / ( 2 * units::Qpow(dx2dy2, std::ratio<5,2>()));
      units::RQuantity<std::ratio<0>, std::ratio<4>, std::ratio<-6>, std::ratio<0>> b = 2 * dx2dy2 * (dddy(t) * dx(t) + dddx(t) * dy(t));
      units::RQuantity<std::ratio<0>, std::ratio<4>, std::ratio<-6>, std::ratio<0>> c = 6 * (ddx(t) * dy(t) - dx(t) * ddy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));

      return a * (b + c); */

      double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t)).getValue();
      double numA = ((dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2).getValue();
      double numB = (3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t))).getValue();
      double num = numA + numB;

      double result = num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
      return result;

    }
    units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-2>, std::ratio<0>> QuinticHermiteSpline::dCurvature2(units::QTime t) {
        return getDCurvature(t) * getDCurvature(t);
    }
    geometry::Rotation2d QuinticHermiteSpline::getHeading(units::QTime t) {
        return geometry::Rotation2d(dx(t).getValue(), dy(t).getValue());
    }
    units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> QuinticHermiteSpline::sumDCurvature2() { // Returns integral of dCurve^2 over the spline
        units::QTime dt = 1.0 / kSamples;
        units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> sum = 0;
        for (units::QTime t = 0; t < units::second; t += dt) {
            sum += (dt * dCurvature2(t));
        }
        return sum;
    }
    units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> QuinticHermiteSpline::sumDCurvature2(std::vector<QuinticHermiteSpline> splines) {
      units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> sum;
        for(QuinticHermiteSpline x : splines) {
            sum += x.sumDCurvature2();
        }
        return sum;
    }
    units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> QuinticHermiteSpline::optimizeSpline(std::vector<QuinticHermiteSpline> *splines) {
        int count = 0;
        units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> prev = sumDCurvature2(*splines);
        while (count < kMaxIterations) {
            runOptimizationIteration(splines);
            units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> current = sumDCurvature2(*splines);
            //std::printf("%f \n", current);
            if (prev - current < kMinDelta)
                return current;
            prev = current;
            count++;
        }
        return prev;
    }
    void QuinticHermiteSpline::runOptimizationIteration(std::vector<QuinticHermiteSpline> *splines) {
        if (splines->size() <= 1) {
            return;
        }

        ControlPoint controlPoints[splines->size() - 1];
        units::Number magnitude = 0;

        for (int i = 0; i < splines->size() - 1; ++i) {
            //don't try to optimize colinear points
            if (splines->at(i).getStartPose().IsLinear(splines->at(i+1).getStartPose()) || splines->at(i).getEndPose().IsLinear(splines->at(i+1).getEndPose())) {
                continue;
            }

            units::RQuantity<std::ratio<0>, std::ratio<-2>, std::ratio<-1>, std::ratio<0>> original = QuinticHermiteSpline::sumDCurvature2(*splines);
            QuinticHermiteSpline temp = splines->at(i);
            QuinticHermiteSpline temp1 = splines->at(i+1);

            //calculate partial derivatives of sumDCurvature2
            splines->at(i) = QuinticHermiteSpline(temp.x0_, temp.x1_, temp.dx0_, temp.dx1_, temp.ddx0_, temp.ddx1_ + kEpsilon.getValue(), temp.y0_, temp.y1_, temp.dy0_, temp.dy1_, temp.ddy0_, temp.ddy1_);
            splines->at(i+1) = QuinticHermiteSpline(temp1.x0_, temp1.x1_, temp1.dx0_, temp1.dx1_, temp1.ddx0_ + kEpsilon.getValue(), temp1.ddx1_, temp1.y0_, temp1.y1_, temp1.dy0_, temp1.dy1_, temp1.ddy0_, temp1.ddy1_);
            controlPoints[i].ddx = (sumDCurvature2(*splines) - original) / kEpsilon;
            splines->at(i) =  QuinticHermiteSpline(temp.x0_, temp.x1_, temp.dx0_, temp.dx1_, temp.ddx0_, temp.ddx1_, temp.y0_, temp.y1_, temp.dy0_, temp.dy1_, temp.ddy0_, temp.ddy1_ + kEpsilon.getValue());
            splines->at(i+1)= QuinticHermiteSpline(temp1.x0_, temp1.x1_, temp1.dx0_, temp1.dx1_, temp1.ddx0_, temp1.ddx1_, temp1.y0_, temp1.y1_, temp1.dy0_, temp1.dy1_, temp1.ddy0_ + kEpsilon.getValue(), temp1.ddy1_);
            controlPoints[i].ddy = (sumDCurvature2(*splines) - original) / kEpsilon;

            splines->at(i) =  temp;
            splines->at(i+1) = temp1;
            magnitude += controlPoints[i].ddx * controlPoints[i].ddx + controlPoints[i].ddy * controlPoints[i].ddy;
        }

        //minimize along the direction of the gradient
        //first calculate 3 points along the direction of the gradient
        geometry::Translation2d p1, p2, p3;
        p2 = geometry::Translation2d(0, sumDCurvature2(*splines).getValue()); //middle point is at the current location

        for (int i = 0; i < splines->size() - 1; ++i) { //first point is offset from the middle location by -stepSize
            //don't try to optimize colinear points
            if (splines->at(i).getStartPose().IsLinear(splines->at(i+1).getStartPose()) || splines->at(i).getEndPose().IsLinear(splines->at(i+1).getEndPose())) {
                continue;
            }

            //normalize to step size
            controlPoints[i].ddx *= (kStepSize / units::Qsqrt(magnitude)).getValue();
            controlPoints[i].ddy *= (kStepSize / units::Qsqrt(magnitude)).getValue();

            //move opposite the gradient by step size amount
            splines->at(i).ddx1_ -= controlPoints[i].ddx.getValue();
            splines->at(i).ddy1_ -= controlPoints[i].ddy.getValue();
            splines->at(i + 1).ddx0_ -= controlPoints[i].ddx.getValue();
            splines->at(i + 1).ddy0_ -= controlPoints[i].ddy.getValue();

            //recompute the spline's coefficients to account for new second derivatives
            splines->at(i).computeCoefficients();
            splines->at(i + 1).computeCoefficients();
        }

        p1 = geometry::Translation2d(-1 * kStepSize, sumDCurvature2(*splines).getValue());
        for (int i = 0; i < splines->size() - 1; ++i) { //last point is offset from the middle location by +stepSize
            //don't try to optimize colinear points
            if (splines->at(i).getStartPose().IsLinear(splines->at(i+1).getStartPose()) || splines->at(i).getEndPose().IsLinear(splines->at(i+1).getEndPose())) {
                continue;
            }
            //move along the gradient by 2 times the step size amount (to return to original location and move by 1
            // step)
            splines->at(i).ddx1_ += 2 * controlPoints[i].ddx.getValue();
            splines->at(i).ddy1_ += 2 * controlPoints[i].ddy.getValue();
            splines->at(i + 1).ddx0_ += 2 * controlPoints[i].ddx.getValue();
            splines->at(i + 1).ddy0_ += 2 * controlPoints[i].ddy.getValue();

            //recompute the spline's coefficients to account for new second derivatives
            splines->at(i).computeCoefficients();
            splines->at(i + 1).computeCoefficients();
        }

        p3 = geometry::Translation2d(kStepSize, sumDCurvature2(*splines).getValue());
        units::QLength stepSize = fitParabola(p1, p2, p3); //approximate step size to minimize sumDCurvature2 along the gradient

        for (int i = 0; i < splines->size() - 1; ++i) {
            //don't try to optimize colinear points
            if (splines->at(i).getStartPose().IsLinear(splines->at(i+1).getStartPose()) || splines->at(i).getEndPose().IsLinear(splines->at(i+1).getEndPose())) {
                continue;
            }
            //move by the step size calculated by the parabola fit (+1 to offset for the final transformation to find
            // p3)
            controlPoints[i].ddx *= 1.0 * units::num + stepSize / kStepSize;
            controlPoints[i].ddy *= 1.0 * units::num + stepSize / kStepSize;

            splines->at(i).ddx1_ += controlPoints[i].ddx.getValue();
            splines->at(i).ddy1_ += controlPoints[i].ddy.getValue();
            splines->at(i + 1).ddx0_ += controlPoints[i].ddx.getValue();
            splines->at(i + 1).ddy0_ += controlPoints[i].ddy.getValue();

            //recompute the spline's coefficients to account for new second derivatives
            splines->at(i).computeCoefficients();
            splines->at(i + 1).computeCoefficients();
        }

    }
    units::QLength QuinticHermiteSpline::fitParabola(geometry::Translation2d p1, geometry::Translation2d p2, geometry::Translation2d p3) {
        double A = (p3.x() * (p2.y() - p1.y()) + p2.x() * (p1.y() - p3.y()) + p1.x() * (p3.y() - p2.y())).getValue();
        double B = (p3.x() * p3.x() * (p1.y() - p2.y()) + p2.x() * p2.x() * (p3.y() - p1.y()) + p1.x() * p1.x() *
                                                                                                (p2.y() - p3.y())).getValue();
        return -B / (2 * A);
    }


}