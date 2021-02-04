//
// Created by alexweiss on 7/3/19.
//

#include "testSpline.hpp"
#include "../../../lib/geometry/Rotation2d.hpp"
#include "../../../lib/geometry/Translation2d.hpp"
#include "../../../lib/geometry/Pose2d.hpp"
#include "../../../lib/geometry/Twist2d.hpp"
#include "../../../lib/spline/CubicHermiteSpline.hpp"
#include "../../../lib/spline/QuinticHermiteSpline.hpp"
#include "../../../lib/spline/SplineGenerator.hpp"
#include "main.h"
#include <cmath>
#include <vector>
#include <stdio.h>


namespace test {
    void testSpline::testCubicSpline() {
        geometry::Pose2d start(geometry::Translation2d(0,0), geometry::Rotation2d::fromDegrees(90));
        geometry::Pose2d end(geometry::Translation2d(5,10), geometry::Rotation2d::fromDegrees(0));
        spline::CubicHermiteSpline spline(start, end);

        std::printf("%f %f %f %f \n", spline.ax_, spline.bx_, spline.cx_, spline.dx_);
        std::printf("%f %f %f %f \n", spline.ay_, spline.by_, spline.cy_, spline.dy_);
        std::printf("%f %f \n", spline.getCurvature(0), spline.getVelocity(0));
        std::printf("%f %f \n", spline.getCurvature(1), spline.getVelocity(1));
    }
    void testSpline::testQuinticSpline() {
        geometry::Pose2d a(geometry::Translation2d(0, 100), geometry::Rotation2d::fromDegrees(270));
        geometry::Pose2d b(geometry::Translation2d(50, 0), geometry::Rotation2d::fromDegrees(0));
        geometry::Pose2d c(geometry::Translation2d(100, 100), geometry::Rotation2d::fromDegrees(90));

        std::vector<spline::QuinticHermiteSpline> splines;
        splines.push_back(spline::QuinticHermiteSpline(a, b));
        splines.push_back(spline::QuinticHermiteSpline(b, c));

        long startTime = pros::millis();
        double x = spline::QuinticHermiteSpline::optimizeSpline(&splines).getValue();
        std::printf("%f \n", x);
        assertTrue(x < 0.014);
        std::printf("Optimization time (ms): %d \n" , (pros::millis() - startTime));

        geometry::Pose2d d(geometry::Translation2d(0, 0), geometry::Rotation2d::fromDegrees(90));
        geometry::Pose2d e(geometry::Translation2d(0, 50), geometry::Rotation2d::fromDegrees(0));
        geometry::Pose2d f(geometry::Translation2d(100, 0), geometry::Rotation2d::fromDegrees(90));
        geometry::Pose2d g(geometry::Translation2d(100, 100), geometry::Rotation2d::fromDegrees(0));

        std::vector<spline::QuinticHermiteSpline> splines1;
        splines1.push_back(spline::QuinticHermiteSpline(d, e));
        splines1.push_back(spline::QuinticHermiteSpline(e, f));
        splines1.push_back(spline::QuinticHermiteSpline(f, g));

        startTime = pros::millis();
        assertTrue(spline::QuinticHermiteSpline::optimizeSpline(&splines1).getValue() < 0.16);
        std::printf("Optimization time (ms): %d \n", (pros::millis() - startTime));


        geometry::Pose2d h(geometry::Translation2d(0, 0), geometry::Rotation2d::fromDegrees(0));
        geometry::Pose2d i(geometry::Translation2d(50, 0), geometry::Rotation2d::fromDegrees(0));
        geometry::Pose2d j(geometry::Translation2d(100, 50), geometry::Rotation2d::fromDegrees(45));
        geometry::Pose2d k(geometry::Translation2d(150, 0), geometry::Rotation2d::fromDegrees(270));
        geometry::Pose2d l(geometry::Translation2d(150, -50), geometry::Rotation2d::fromDegrees(270));

        std::vector<spline::QuinticHermiteSpline> splines2;
        splines2.push_back(spline::QuinticHermiteSpline(h, i));
        splines2.push_back(spline::QuinticHermiteSpline(i, j));
        splines2.push_back(spline::QuinticHermiteSpline(j, k));
        splines2.push_back(spline::QuinticHermiteSpline(k, l));

        startTime = pros::millis();
        assertTrue(spline::QuinticHermiteSpline::optimizeSpline(&splines2).getValue() < 0.05);
        assertEquals(splines2.at(0).getCurvature(1.0), 0.0, EPSILON);
        assertEquals(splines2.at(2).getCurvature(1.0), 0.0, EPSILON);
        std::printf("Optimization time (ms): %d \n", (pros::millis() - startTime));
    }

    void testSpline::testSplineGenerator() {
        geometry::Pose2d p1(geometry::Translation2d(0, 0), geometry::Rotation2d());
        geometry::Pose2d p2(geometry::Translation2d(15, 10), geometry::Rotation2d(1, -5));
        spline::Spline *s = new spline::QuinticHermiteSpline(p1, p2);

        std::vector<geometry::Pose2dWithCurvature> samples = spline::SplineGenerator::parameterizeSpline(s);

        units::QLength arclength = 0;
        geometry::Pose2dWithCurvature cur_pose = samples.at(0);
        for (geometry::Pose2dWithCurvature sample : samples) {
            geometry::Twist2d t = geometry::Pose2d::log(cur_pose.pose().inverse().transformBy(sample.pose()));
            arclength += t.dx_;
            cur_pose = sample;
        }

        assertEquals(cur_pose.translation().x().getValue(), 15.0, EPSILON);
        assertEquals(cur_pose.translation().y().getValue(), 10.0, EPSILON);
        assertEquals(cur_pose.rotation().getDegrees(), -78.69006752597981, EPSILON);
        assertEquals(arclength.getValue(), 23.225668846151, EPSILON);
    }

    void testSpline::testDCurve() {
      geometry::Pose2d a(geometry::Translation2d(0, 100), geometry::Rotation2d::fromDegrees(270));
      geometry::Pose2d b(geometry::Translation2d(50, 0), geometry::Rotation2d::fromDegrees(0));

      spline::QuinticHermiteSpline basic(a, b);
      units::QTime dt = 0.01;
      units::QTime t = 0;
      while(t < 1.0*units::second) {
        printf("(%f, %f, %f)\n", t.getValue(), basic.getCurvature(t).getValue(), basic.getDCurvature(t).getValue());
        t += dt;
      }

      printf("%f", basic.getDCurvature(0.5 * units::second).getValue());

    }
}