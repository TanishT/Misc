//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYUTIL_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYUTIL_HPP_

#include <vector>
#include <cmath>
#include "../geometry/interfaces/IPose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"
#include "../geometry/Pose2d.hpp"
#include "../utility/Units.hpp"
#include "IPathFollower.hpp"
#include "Trajectory.hpp"
#include "TrajectoryView.hpp"
#include "timing/TimedState.hpp"
#include "../geometry/interfaces/IPose2d.hpp"
#include "../spline/SplineGenerator.hpp"
#include "../spline/QuinticHermiteSpline.hpp"
#include "../utility/Utility.hpp"





namespace trajectory {
  class TrajectoryUtil {
    public:
      template <class S>
      static Trajectory<S> mirror(Trajectory<S> trajectory) {
        static_assert(std::is_base_of<geometry::IPose2d<S>, S>::value, "S must derive from State");
        std::vector<S> waypoints;
        for (int i = 0; i < trajectory.length()(); ++i) {
          waypoints.push_back(trajectory.getState(i).mirror());
        }
        return Trajectory<S>(waypoints);
      }

      template <class S>
      static Trajectory<TimedState<S>> mirrorTimed(Trajectory<TimedState<S>> trajectory)  {
        std::vector<TimedState<S>> waypoints;
        for (int i = 0; i < trajectory.length(); ++i) {
          TimedState<S> timed_state = trajectory.getState(i);
          waypoints.push_back(TimedState<S>(timed_state.state().mirror(), timed_state.t(), timed_state.velocity(), timed_state.acceleration()));
        }
        return Trajectory<TimedState<S>>(waypoints);
      }

      template <class S>
      static Trajectory<S> transform(Trajectory<S> trajectory, geometry::Pose2d transform)  {
        std::vector<TimedState<S>> waypoints;
        for (int i = 0; i < trajectory.length(); ++i) {
          waypoints.push_back(trajectory.getState(i).transformBy(transform));
        }
        return Trajectory<TimedState<S>>(waypoints);
      }

      template <class S>
      static Trajectory<S> resample(TrajectoryView<S> trajectory_view, double interval)  {
        if (interval <= EPSILON) {
          return new Trajectory<S>();
        }
        int num_states = (int) std::ceil((trajectory_view.last_interpolant() - trajectory_view.first_interpolant()) / interval);
        std::vector<S> states;

        for (int i = 0; i < num_states; ++i) {
          states.push_back(trajectory_view.sample(i * interval + trajectory_view.first_interpolant()).state());
        }
        return Trajectory<S>(states);
      }


      template <class S>
      static Trajectory<geometry::Pose2dWithCurvature> trajectoryFromSplines(std::vector<S> splines, units::QLength maxDx, units::QLength maxDy, units::Angle maxDTheta) {
        std::vector<geometry::Pose2dWithCurvature> x = spline::SplineGenerator::parameterizeSplines(&splines, maxDx, maxDy, maxDTheta);
        return Trajectory<geometry::Pose2dWithCurvature>(x);
      }


      static Trajectory<geometry::Pose2dWithCurvature> trajectoryFromPathFollower(IPathFollower* path_follower,
          geometry::Pose2dWithCurvature start_state, units::QLength step_size, units::QDCurvatureDs dcurvature_limit);

      static Trajectory<geometry::Pose2dWithCurvature> trajectoryFromSplineWaypoints(
          std::vector<geometry::Pose2d> waypoints, units::QLength maxDx, units::QLength maxDy, units::Angle maxDTheta);

  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYUTIL_HPP_
