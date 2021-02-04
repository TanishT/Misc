//
// Created by alexweiss on 8/2/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMINGUTIL_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMINGUTIL_HPP_

#include <cmath>
#include <stdio.h>
#include <vector>
#include <string>
#include <type_traits>
#include "../../geometry/interfaces/State.hpp"
#include "../Trajectory.hpp"
#include "../DistanceView.hpp"
#include "TimingConstraint.hpp"
#include "TimedState.hpp"

namespace trajectory {
  class TimingUtil {
   public:

    template<class S>
    class ConstrainedState : public S {
      static_assert(std::is_base_of<geometry::State<S>, S>::value, "S is not derived from State");
     public:
      S state;
      units::QLength distance;
      units::QSpeed max_velocity;
      units::QAcceleration min_acceleration;
      units::QAcceleration max_acceleration;
    };

    template<class S>
    static Trajectory<TimedState<S>> reverseTimed(Trajectory<TimedState<S>> forward) {
      std::vector<TimedState<S>> states;
      int num_states = forward.length();
      units::QTime final_t = forward.getState(num_states - 1).t();
      geometry::Pose2d flip = geometry::Pose2d::fromRotation(geometry::Rotation2d(-1, 0));
      for(int i = num_states - 1; i >= 0; i--) {
        TimedState<S> state = forward.getState(i);
        TimedState<S> newState(state.state(), final_t - state.t(), -1*state.velocity(), -1*state.acceleration());
        states.push_back(newState);
      }
      return Trajectory<TimedState<S>>(states);
    }

    template<class S>
    static Trajectory<TimedState<S>> timeParameterizeTrajectory(
        bool reverse,
        DistanceView<S> distance_view,
        units::QLength step_size,
        std::vector<TimingConstraint<S>*> constraints,
        units::QSpeed start_velocity,
        units::QSpeed end_velocity,
        units::QSpeed max_velocity,
        units::QAcceleration max_abs_acceleration)  {

      int num_states = (int) std::ceil((distance_view.last_interpolant() / step_size + 1).getValue());
      std::vector<S> states;
      for (int i = 0; i < num_states; ++i) {
        states.push_back(S(distance_view.sample(MIN(i * step_size, distance_view.last_interpolant()*units::metre).getValue()).state()));
      }
      return timeParameterizeTrajectory(reverse, states, constraints, start_velocity, end_velocity,
                                        max_velocity, max_abs_acceleration);
    }

    template<class S>
    static Trajectory<TimedState<S>> timeParameterizeTrajectory(
        bool reverse,
        std::vector<S> states,
        std::vector<TimingConstraint<S>*> constraints,
        units::QSpeed start_velocity,
        units::QSpeed end_velocity,
        units::QSpeed max_velocity_global,
        units::QAcceleration max_abs_acceleration)  {



      std::vector<ConstrainedState<S>*> constraint_states;


      // Forward pass. We look at pairs of consecutive states, where the start state has already been velocity
      // parameterized (though we may adjust the velocity downwards during the backwards pass). We wish to find an
      // acceleration that is admissible at both the start and end state, as well as an admissible end velocity. If
      // there is no admissible end velocity or acceleration, we set the end velocity to the state's maximum allowed
      // velocity and will repair the acceleration during the backward pass (by slowing down the predecessor).
      ConstrainedState<S>* predecessor = new ConstrainedState<S>();
      predecessor->state = states.at(0);
      predecessor->distance = -0.001;
      predecessor->max_velocity = start_velocity;
      predecessor->min_acceleration = -1 * max_abs_acceleration;
      predecessor->max_acceleration = max_abs_acceleration;

      for (int i = 0; i < states.size(); ++i) {
        //printf("i %d\n", i);
        // Add the new state.
        constraint_states.push_back(new ConstrainedState<S>());
        constraint_states.at(i)->state = S(states.at(i));
        units::QLength ds = constraint_states.at(i)->state.distance(predecessor->state);
        constraint_states.at(i)->distance = ds + predecessor->distance;

        // We may need to iterate to find the maximum end velocity and common acceleration, since acceleration
        // limits may be a function of velocity.

        while (true) {
          // Enforce global max velocity and max reachable velocity by global acceleration limit.

          /* if(std::isnan(predecessor->max_acceleration.getValue()))
            predecessor->max_acceleration = 0; */

          // vf = sqrt(vi^2 + 2*a*d)
          constraint_states.at(i)->max_velocity = MIN(max_velocity_global,
                                                      units::Qsqrt(units::Qabs(predecessor->max_velocity * predecessor->max_velocity + 2.0 * predecessor->max_acceleration * ds)));


          constraint_states.at(i)->min_acceleration = -1 * max_abs_acceleration;
          constraint_states.at(i)->max_acceleration = max_abs_acceleration;

          for (TimingConstraint<S>* constraint : constraints) {
            constraint_states.at(i)->max_velocity = MIN(constraint_states.at(i)->max_velocity,
                                                        constraint->getMaxVelocity(constraint_states.at(i)->state));
          }


          for (TimingConstraint<S>*constraint : constraints) {
            physics::DifferentialDrive::MinMaxAcceleration min_max_accel = constraint->getMinMaxAcceleration(
                constraint_states.at(i)->state,
                (reverse ? -1.0 : 1.0) * constraint_states.at(i)->max_velocity);

            constraint_states.at(i)->min_acceleration = MAX(constraint_states.at(i)->min_acceleration,
                                                            reverse ? -1*min_max_accel.max_acceleration() : min_max_accel.min_acceleration());
            constraint_states.at(i)->max_acceleration = MIN(constraint_states.at(i)->max_acceleration,
                                                            reverse ? -1*min_max_accel.min_acceleration() : min_max_accel.max_acceleration());
          }


          units::QAcceleration actual_acceleration = (constraint_states.at(i)->max_velocity * constraint_states.at(i)->max_velocity
              - predecessor->max_velocity * predecessor->max_velocity) / (2.0 * ds);

          if (constraint_states.at(i)->max_acceleration < actual_acceleration - EPSILON) {
            predecessor->max_acceleration = constraint_states.at(i)->max_acceleration;
          } else {
            if (actual_acceleration > predecessor->min_acceleration + EPSILON) {
              predecessor->max_acceleration = actual_acceleration;
            }
            // If actual acceleration is less than predecessor min accel, we will repair during the backward
            // pass.
            break;
          }
        }

        // System.out.println("i: " + i + ", " + constraint_state.toString());
        predecessor = constraint_states.at(i);
      }
      // Backward pass.

      ConstrainedState<S>* successor = new ConstrainedState<S>();
      successor->state = states.at(states.size() - 1);
      successor->distance = constraint_states.at(states.size() - 1)->distance;
      successor->max_velocity = end_velocity;
      successor->min_acceleration = -1 * max_abs_acceleration;
      successor->max_acceleration = max_abs_acceleration;
      for (int i = states.size() - 1; i >= 0; --i) {
        units::QLength ds = constraint_states.at(i)->distance - successor->distance; // will be negative.

        while (true) {
          // Enforce reverse max reachable velocity limit.
          // vf = sqrt(vi^2 + 2*a*d), where vi = successor.
          units::QSpeed new_max_velocity = units::Qsqrt(units::Qabs(successor->max_velocity * successor->max_velocity
                                                                        + 2.0 * successor->min_acceleration * ds));
          if (new_max_velocity >= constraint_states.at(i)->max_velocity) {
            // No new limits to impose.
            break;
          }
          constraint_states.at(i)->max_velocity = new_max_velocity;
          if (std::isnan(constraint_states.at(i)->max_velocity.getValue())) {
            printf("you might have an really big issue - TimingUtil::162\n");
          }

          // Now check all acceleration constraints with the lower max velocity.
          for (TimingConstraint<S>* constraint : constraints) {
            physics::DifferentialDrive::MinMaxAcceleration min_max_accel = constraint->getMinMaxAcceleration(
                constraint_states.at(i)->state,
                (reverse ? -1.0 : 1.0) * constraint_states.at(i)->max_velocity);
            if (!min_max_accel.valid()) {
              printf("you might have an really big issue - TimingUtil::171\n");
            }
            constraint_states.at(i)->min_acceleration = MAX(constraint_states.at(i)->min_acceleration,
                                                            (reverse ? -1 * min_max_accel.max_acceleration() : min_max_accel.min_acceleration()));
            constraint_states.at(i)->max_acceleration = MIN(constraint_states.at(i)->max_acceleration,
                                                            (reverse ? -1 * min_max_accel.min_acceleration() : min_max_accel.max_acceleration()));
          }
          if (constraint_states.at(i)->min_acceleration > constraint_states.at(i)->max_acceleration) {
            printf("you might have an really big issue - TimingUtil::179\n");
          }

          if (ds.getValue() > EPSILON) {
            break;
          }
          // If the min acceleration for this constraint state is more conservative than what we have applied, we
          // need to reduce the min accel and try again.
          // TODO: Simply using the new min acceleration is guaranteed to be valid, but may be too conservative.
          // Doing a search would be better.
          units::QAcceleration actual_acceleration = (constraint_states.at(i)->max_velocity * constraint_states.at(i)->max_velocity
              - successor->max_velocity * successor->max_velocity) / (2.0 * ds);
          if (constraint_states.at(i)->min_acceleration > actual_acceleration - EPSILON) {
            successor->min_acceleration = constraint_states.at(i)->min_acceleration;
          } else {
            successor->min_acceleration = actual_acceleration;
            break;
          }
        }
        successor = constraint_states.at(i);
      }

      std::vector<TimedState<S>> timed_states;

      units::QTime t = 0.0;
      units::QLength s = 0.0;
      units::QSpeed v = 0.0;
      for (int i = 0; i < states.size(); ++i) {
        // Advance t.
        units::QLength ds = constraint_states.at(i)->distance - s;
        units::QAcceleration accel = (constraint_states.at(i)->max_velocity * constraint_states.at(i)->max_velocity - v * v) / (2.0 * ds);
        units::QTime dt = 0.0;
        if (i > 0) {
          timed_states.at(i - 1).set_acceleration(reverse ? -1*accel : accel);
          if (std::fabs(accel.getValue()) > EPSILON) {
            dt = (constraint_states.at(i)->max_velocity - v) / accel;
          } else if (std::fabs(v.getValue()) > EPSILON) {
            dt = ds / v;
          } else {
            printf("you might have an really big issue - TimingUtil::217\n");
          }
        }
        t += dt;
        if (std::isnan(t.getValue()) || std::isinf(t.getValue())) {
          printf("you might have an really big issue - TimingUtil::223\n");
        }

        v = constraint_states.at(i)->max_velocity;
        s = constraint_states.at(i)->distance;
        timed_states.push_back(TimedState<S>(constraint_states.at(i)->state, t, reverse ? -1*v : v, reverse ? -1*accel : accel));
      }


      return Trajectory<TimedState<S>>(timed_states);
    }
  };
}
#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMINGUTIL_HPP_
