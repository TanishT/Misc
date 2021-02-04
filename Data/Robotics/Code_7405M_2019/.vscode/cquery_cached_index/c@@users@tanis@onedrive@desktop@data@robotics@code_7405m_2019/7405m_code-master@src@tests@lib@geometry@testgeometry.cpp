//
// Created by alexweiss on 7/3/19.
//

#include "testGeometry.hpp"
#include "../../testBase.hpp"
#include "../../../lib/geometry/Rotation2d.hpp"
#include "../../../lib/geometry/Translation2d.hpp"
#include "../../../lib/geometry/Pose2d.hpp"
#include "../../../lib/geometry/Twist2d.hpp"
#include <cmath>
#include <stdio.h>

namespace test {
    void testGeometry::testRotation2d() {
        geometry::Rotation2d rot1;
        assertEquals(1, rot1.cos(), EPSILON);
        assertEquals(0, rot1.sin(), EPSILON);
        assertEquals(0, rot1.tan(), EPSILON);
        assertEquals(0, rot1.getDegrees(), EPSILON);
        assertEquals(0, rot1.getRadians(), EPSILON);

        rot1 = geometry::Rotation2d(1, 1);
        assertEquals(std::sqrt(2.0) / 2, rot1.cos(), EPSILON);
        assertEquals(std::sqrt(2.0) / 2, rot1.sin(), EPSILON);
        assertEquals(1, rot1.tan(), EPSILON);
        assertEquals(45, rot1.getDegrees(), EPSILON);
        assertEquals(M_PI / 4, rot1.getRadians(), EPSILON);

      rot1 = geometry::Rotation2d::fromRadians(M_PI / 2);
      assertEquals(0, rot1.cos(), EPSILON);
      assertEquals(1, rot1.sin(), EPSILON);
      assertTrue(1 / EPSILON < rot1.tan());
      assertEquals(90, rot1.getDegrees(), EPSILON);
      assertEquals(M_PI / 2, rot1.getRadians(), EPSILON);

      rot1 = geometry::Rotation2d::fromDegrees(270);
      assertEquals(0, rot1.cos(), EPSILON);
      assertEquals(-1, rot1.sin(), EPSILON);
      assertTrue(-1 / EPSILON > rot1.tan());
      assertEquals(-90, rot1.getDegrees(), EPSILON);
      assertEquals(-M_PI / 2, rot1.getRadians(), EPSILON);

      // Test inversion
      rot1 = geometry::Rotation2d::fromDegrees(270);
      geometry::Rotation2d rot2 = rot1.inverse();
      assertEquals(0, rot2.cos(), EPSILON);
      assertEquals(1, rot2.sin(), EPSILON);
      assertTrue(1 / EPSILON < rot2.tan());
      assertEquals(90, rot2.getDegrees(), EPSILON);
      assertEquals(M_PI / 2, rot2.getRadians(), EPSILON);

      rot1 = geometry::Rotation2d::fromDegrees(1);
      rot2 = rot1.inverse();
      assertEquals(rot1.cos(), rot2.cos(), EPSILON);
      assertEquals(-rot1.sin(), rot2.sin(), EPSILON);
      assertEquals(-1, rot2.getDegrees(), EPSILON);

      // Test rotateBy
      rot1 = geometry::Rotation2d::fromDegrees(45);
      rot2 = geometry::Rotation2d::fromDegrees(45);
      geometry::Rotation2d rot3 = rot1.rotateBy(rot2);
      assertEquals(0, rot3.cos(), EPSILON);
      assertEquals(1, rot3.sin(), EPSILON);
      assertTrue(1 / EPSILON < rot3.tan());
      assertEquals(90, rot3.getDegrees(), EPSILON);
      assertEquals(M_PI / 2, rot3.getRadians(), EPSILON);

      rot1 = geometry::Rotation2d::fromDegrees(45);
      rot2 = geometry::Rotation2d::fromDegrees(-45);
      rot3 = rot1.rotateBy(rot2);
      assertEquals(1, rot3.cos(), EPSILON);
      assertEquals(0, rot3.sin(), EPSILON);
      assertEquals(0, rot3.tan(), EPSILON);
      assertEquals(0, rot3.getDegrees(), EPSILON);
      assertEquals(0, rot3.getRadians(), EPSILON);

      // A rotation times its inverse should be the identity
      geometry::Rotation2d identity = geometry::Rotation2d();
      rot1 = geometry::Rotation2d::fromDegrees(21.45);
      rot2 = rot1.rotateBy(rot1.inverse());
      assertEquals(identity.cos(), rot2.cos(), EPSILON);
      assertEquals(identity.sin(), rot2.sin(), EPSILON);
      assertEquals(identity.getDegrees(), rot2.getDegrees(), EPSILON);

      // Test interpolation
      rot1 = geometry::Rotation2d::fromDegrees(45);
      rot2 = geometry::Rotation2d::fromDegrees(135);
      rot3 = rot1.interpolate(rot2, .5);
      assertEquals(90, rot3.getDegrees(), EPSILON);

      rot1 = geometry::Rotation2d::fromDegrees(45);
      rot2 = geometry::Rotation2d::fromDegrees(135);
      rot3 = rot1.interpolate(rot2, .75);
      assertEquals(112.5, rot3.getDegrees(), EPSILON);

      rot1 = geometry::Rotation2d::fromDegrees(45);
      rot2 = geometry::Rotation2d::fromDegrees(-45);
      rot3 = rot1.interpolate(rot2, .5);
      assertEquals(0, rot3.getDegrees(), EPSILON);

      rot1 = geometry::Rotation2d::fromDegrees(45);
      rot2 = geometry::Rotation2d::fromDegrees(45);
      rot3 = rot1.interpolate(rot2, .5);
      assertEquals(45, rot3.getDegrees(), EPSILON);

      rot1 = geometry::Rotation2d::fromDegrees(45);
      rot2 = geometry::Rotation2d::fromDegrees(45);
      rot3 = rot1.interpolate(rot2, .5);
      assertEquals(45, rot3.getDegrees(), EPSILON);

      // Test parallel.
      rot1 = geometry::Rotation2d::fromDegrees(45);
      rot2 = geometry::Rotation2d::fromDegrees(45);
      assertTrue(rot1.isParallel(rot2));

      rot1 = geometry::Rotation2d::fromDegrees(45);
      rot2 = geometry::Rotation2d::fromDegrees(-45);
      assertFalse(rot1.isParallel(rot2));

      rot1 = geometry::Rotation2d::fromDegrees(45);
      rot2 = geometry::Rotation2d::fromDegrees(-135);
      assertTrue(rot1.isParallel(rot2));
    }

    void testGeometry::testTranslation2d() {
        geometry::Translation2d pos1;
        assertEquals(0, pos1.x().getValue(), EPSILON);
        assertEquals(0, pos1.y().getValue(), EPSILON);
        assertEquals(0, pos1.norm().getValue(), EPSILON);

        pos1 = geometry::Translation2d(3, 4);
        assertEquals(3, pos1.x().getValue(), EPSILON);
        assertEquals(4, pos1.y().getValue(), EPSILON);
        assertEquals(5, pos1.norm().getValue(), EPSILON);

        // Test inversion
        pos1 = geometry::Translation2d(3.152, 4.1666);
        geometry::Translation2d pos2 = pos1.inverse();
        assertEquals(-1*pos1.x().getValue(), pos2.x().getValue(), EPSILON);
        assertEquals(-1*pos1.y().getValue(), pos2.y().getValue(), EPSILON);
        assertEquals(pos1.norm().getValue(), pos2.norm().getValue(), EPSILON);

        // Test rotateBy
        pos1 = geometry::Translation2d(2, 0);
        geometry::Rotation2d rot1 = geometry::Rotation2d::fromDegrees(90);
        pos2 = pos1.rotateBy(rot1);
        assertEquals(0, pos2.x().getValue(), EPSILON);
        assertEquals(2, pos2.y().getValue(), EPSILON);
        assertEquals(pos1.norm().getValue(), pos2.norm().getValue(), EPSILON);

        pos1 = geometry::Translation2d(2, 0);
        rot1 = geometry::Rotation2d::fromDegrees(-45);
        pos2 = pos1.rotateBy(rot1);
        assertEquals(std::sqrt(2), pos2.x().getValue(), EPSILON);
        assertEquals(-std::sqrt(2), pos2.y().getValue(), EPSILON);
        assertEquals(pos1.norm().getValue(), pos2.norm().getValue(), EPSILON);

        // Test translateBy
        pos1 = geometry::Translation2d(2, 0);
        pos2 = geometry::Translation2d(-2, 1);
        geometry::Translation2d pos3 = pos1.translateBy(pos2);
        assertEquals(0, pos3.x().getValue(), EPSILON);
        assertEquals(1, pos3.y().getValue(), EPSILON);
        assertEquals(1, pos3.norm().getValue(), EPSILON);

        // A translation times its inverse should be the identity
        geometry::Translation2d identity = geometry::Translation2d();
        pos1 = geometry::Translation2d(2.16612, -23.55);
        pos2 = pos1.translateBy(pos1.inverse());
        assertEquals(identity.x().getValue(), pos2.x().getValue(), EPSILON);
        assertEquals(identity.y().getValue(), pos2.y().getValue(), EPSILON);
        assertEquals(identity.norm().getValue(), pos2.norm().getValue(), EPSILON);

        // Test interpolation
        pos1 = geometry::Translation2d(0, 1);
        pos2 = geometry::Translation2d(10, -1);
        pos3 = pos1.interpolate(pos2, .5);
        assertEquals(5, pos3.x().getValue(), EPSILON);
        assertEquals(0, pos3.y().getValue(), EPSILON);

        pos1 = geometry::Translation2d(0, 1);
        pos2 = geometry::Translation2d(10, -1);
        pos3 = pos1.interpolate(pos2, .75);
        assertEquals(7.5, pos3.x().getValue(), EPSILON);
        assertEquals(-.5, pos3.y().getValue(), EPSILON);
    }

    void testGeometry::testPose2d() {
        geometry::Pose2d pose1 = geometry::Pose2d();
        assertEquals(0, pose1.translation().x().getValue(), EPSILON);
        assertEquals(0, pose1.translation().y().getValue(), EPSILON);
        assertEquals(0, pose1.rotation().getDegrees(), EPSILON);

        pose1 = geometry::Pose2d(geometry::Translation2d(3, 4), geometry::Rotation2d::fromDegrees(45));
        assertEquals(3, pose1.translation().x().getValue(), EPSILON);
        assertEquals(4, pose1.translation().y().getValue(), EPSILON);
        assertEquals(45, pose1.rotation().getDegrees(), EPSILON);

        // Test transformation
        pose1 = geometry::Pose2d(geometry::Translation2d(3, 4), geometry::Rotation2d::fromDegrees(90));
        geometry::Pose2d pose2 = geometry::Pose2d(geometry::Translation2d(1, 0), geometry::Rotation2d::fromDegrees(0));
        geometry::Pose2d pose3 = pose1.transformBy(pose2);
        assertEquals(3, pose3.translation().x().getValue(), EPSILON);
        assertEquals(5, pose3.translation().y().getValue(), EPSILON);
        assertEquals(90, pose3.rotation().getDegrees(), EPSILON);

        pose1 = geometry::Pose2d(geometry::Translation2d(3, 4), geometry::Rotation2d::fromDegrees(90));
        pose2 = geometry::Pose2d(geometry::Translation2d(1, 0), geometry::Rotation2d::fromDegrees(-90));
        pose3 = pose1.transformBy(pose2);
        assertEquals(3, pose3.translation().x().getValue(), EPSILON);
        assertEquals(5, pose3.translation().y().getValue(), EPSILON);
        assertEquals(0, pose3.rotation().getDegrees(), EPSILON);

        // A pose times its inverse should be the identity
        geometry::Pose2d identity = geometry::Pose2d();
        pose1 = geometry::Pose2d(geometry::Translation2d(3.51512152, 4.23), geometry::Rotation2d::fromDegrees(91.6));
        pose2 = pose1.transformBy(pose1.inverse());
        assertEquals(identity.translation().x().getValue(), pose2.translation().x().getValue(), EPSILON);
        assertEquals(identity.translation().y().getValue(), pose2.translation().y().getValue(), EPSILON);
        assertEquals(identity.rotation().getDegrees(), pose2.rotation().getDegrees(), EPSILON);

        // Test interpolation
        // Movement from pose1 to pose2 is along a circle with radius of 10 units centered at (3, -6)
        pose1 = geometry::Pose2d(geometry::Translation2d(3, 4), geometry::Rotation2d::fromDegrees(90));
        pose2 = geometry::Pose2d(geometry::Translation2d(13, -6), geometry::Rotation2d::fromDegrees(0.0));
        pose3 = pose1.interpolate(pose2, .5);
        double expected_angle_rads = M_PI / 4;
        assertEquals(3.0 + 10.0 * cos(expected_angle_rads), pose3.translation().x().getValue(), EPSILON);
        assertEquals(-6.0 + 10.0 * sin(expected_angle_rads), pose3.translation().y().getValue(), EPSILON);
        assertEquals(expected_angle_rads, pose3.rotation().getRadians(), EPSILON);

        pose1 = geometry::Pose2d(geometry::Translation2d(3, 4), geometry::Rotation2d::fromDegrees(90));
        pose2 = geometry::Pose2d(geometry::Translation2d(13, -6), geometry::Rotation2d::fromDegrees(0.0));
        pose3 = pose1.interpolate(pose2, .75);
        expected_angle_rads = M_PI / 8;
        assertEquals(3.0 + 10.0 * cos(expected_angle_rads), pose3.translation().x().getValue(), EPSILON);
        assertEquals(-6.0 + 10.0 * sin(expected_angle_rads), pose3.translation().y().getValue(), EPSILON);
        assertEquals(expected_angle_rads, pose3.rotation().getRadians(), EPSILON);
    }

    void testGeometry::testTwist() {
        // Exponentiation (integrate twist to obtain a Pose2d)
        geometry::Twist2d twist = geometry::Twist2d(1.0, 0.0, 0.0);
        geometry::Pose2d pose = geometry::Pose2d::exp(twist);
        assertEquals(1.0, pose.translation().x().getValue(), EPSILON);
        assertEquals(0.0, pose.translation().y().getValue(), EPSILON);
        assertEquals(0.0, pose.rotation().getDegrees(), EPSILON);

        // Scaled.
        twist = geometry::Twist2d(1.0, 0.0, 0.0);
        pose = geometry::Pose2d::exp(twist.scaled(2.5));
        assertEquals(2.5, pose.translation().x().getValue(), EPSILON);
        assertEquals(0.0, pose.translation().y().getValue(), EPSILON);
        assertEquals(0.0, pose.rotation().getDegrees(), EPSILON);

        // Logarithm (find the twist to apply to obtain a given Pose2d)

        pose = geometry::Pose2d(geometry::Translation2d(2.0, 2.0), geometry::Rotation2d::fromRadians(M_PI / 2));
        twist = geometry::Pose2d::log(pose);
        assertEquals(M_PI, twist.dx_.getValue(), EPSILON);
        assertEquals(0.0, twist.dy_.getValue(), EPSILON);
        assertEquals(M_PI / 2, twist.dtheta_.getValue(), EPSILON);

        // Logarithm is the inverse of exponentiation.
        geometry::Pose2d new_pose = geometry::Pose2d::exp(twist);
        assertEquals(new_pose.translation().x().getValue(), pose.translation().x().getValue(), EPSILON);
        assertEquals(new_pose.translation().y().getValue(), pose.translation().y().getValue(), EPSILON);
        assertEquals(new_pose.rotation().getDegrees(), pose.rotation().getDegrees(), EPSILON);
    }
}
