//
// Created by alexweiss on 8/10/19.
//

#include "lib/geometry/testGeometry.hpp"
#include "lib/physics/testPhysics.hpp"
#include "lib/spline/testSpline.hpp"
#include "lib/trajectory/testTrajectory.hpp"
#include "lib/trajectory/timing/testTimedState.hpp"

namespace test {
  void testMeecanLib() {
    // testGeometry::testRotation2d();
    // testGeometry::testTranslation2d();
    // testGeometry::testTwist();
    //testGeometry::testPose2d();

    // testPhysics::testDifferentialDrive();
    // testPhysics::testDCMotorTransmission();
    // testPhysics::testDriveCharacterization();

    //testSpline::testSplineGenerator();
    // testSpline::testCubicSpline();
    //testSpline::testQuinticSpline();
    //testSpline::testDCurve();

    //testTrajectory::testDistanceView();
    //testTrajectory::testTrajectoryIterator();
    //testTrajectory::testIntegration();
    //testTrajectory::testTrajectoryClass();
    testTrajectory::realisticConstants();

    // testTimedState::test();
  }
}