//
// Created by alexweiss on 8/3/19.
//

#include <cmath>
#include <iostream>
#include "TrajectoryUtil.hpp"
#include "../geometry/interfaces/IPose2d.hpp"
#include "../spline/SplineGenerator.hpp"
#include "../spline/QuinticHermiteSpline.hpp"
#include "../utility/Utility.hpp"

namespace trajectory {


    Trajectory<geometry::Pose2dWithCurvature> TrajectoryUtil::trajectoryFromPathFollower(IPathFollower* path_follower,
      geometry::Pose2dWithCurvature start_state, units::QLength step_size, units::QDCurvatureDs dcurvature_limit) {

    std::vector<geometry::Pose2dWithCurvature> samples;
    samples.push_back(start_state);
    geometry::Pose2dWithCurvature current_state = start_state;

    while (!path_follower->isDone()) {
      // Get the desired steering command.

      geometry::Twist2d raw_steering_command = path_follower->steer(current_state.pose());

      // Truncate to the step size.
      geometry::Twist2d steering_command = raw_steering_command;
      if (steering_command.norm() > step_size) {
        steering_command = steering_command.scaled((step_size / steering_command.norm()));
      }


      // Apply limits on spatial derivative of curvature, if desired.
      units::QDCurvatureDs dcurvature = ((steering_command.curvature() - current_state.curvature()) / steering_command.norm());
      bool curvature_valid = !std::isnan(dcurvature.getValue()) && !std::isinf(dcurvature.getValue())
          && !std::isnan(current_state.curvature().getValue()) && !std::isinf((current_state.curvature().getValue()));
      if (dcurvature > dcurvature_limit && curvature_valid) {
        double angle = ((dcurvature_limit.getValue() * steering_command.norm().getValue() + current_state.curvature().getValue()) * steering_command.norm().getValue()); // TODO: FIX
        steering_command = geometry::Twist2d(steering_command.dx_, steering_command.dy_, angle * units::radian);
      } else if (dcurvature < -1 * dcurvature_limit && curvature_valid) {
        double angle = ((-1 * dcurvature_limit.getValue() * steering_command.norm().getValue() + current_state.curvature().getValue()) * steering_command.norm().getValue()); // TODO: FIX
        steering_command = geometry::Twist2d(steering_command.dx_, steering_command.dy_, angle * units::radian);
      }


      // Calculate the new state.
      // Use the average curvature over the interval to compute the next state.
      double angle = (current_state.curvature().getValue() + 0.5 * dcurvature.getValue() * steering_command.norm().getValue()) * steering_command.norm().getValue();      geometry::Twist2d average_steering_command = !curvature_valid
                                               ? steering_command
                                               : geometry::Twist2d(steering_command.dx_, steering_command.dy_, angle * units::radian);


      current_state = geometry::Pose2dWithCurvature(
          current_state.pose().transformBy(geometry::Pose2d::exp(average_steering_command)),
          steering_command.curvature());

      if (!path_follower->isDone()) {
        samples.push_back(current_state);
      }

    }

    return Trajectory<geometry::Pose2dWithCurvature>(samples);
  }

  Trajectory<geometry::Pose2dWithCurvature> TrajectoryUtil::trajectoryFromSplineWaypoints(std::vector<geometry::Pose2d> waypoints, units::QLength maxDx, units::QLength maxDy, units::Angle maxDTheta) {
    std::vector<spline::QuinticHermiteSpline> splines;
    for (int i = 1; i < waypoints.size(); ++i) {
      splines.push_back(spline::QuinticHermiteSpline(waypoints.at(i - 1), waypoints.at(i)));
    }
    spline::QuinticHermiteSpline::optimizeSpline(&splines);
    return trajectoryFromSplines(splines, maxDx, maxDy, maxDTheta);
  }

}