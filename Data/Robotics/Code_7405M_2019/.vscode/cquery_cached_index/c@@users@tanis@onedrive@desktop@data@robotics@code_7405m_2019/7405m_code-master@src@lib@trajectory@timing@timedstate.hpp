//
// Created by alexweiss on 7/9/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMEDSTATE_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMEDSTATE_HPP_

#include "../../geometry/interfaces/State.hpp"
#include "../../geometry/Translation2d.hpp"
#include "../../geometry/Rotation2d.hpp"
#include "../../geometry/Pose2d.hpp"
#include "../../geometry/Pose2dWithCurvature.hpp"
#include "../../utility/Utility.hpp"
#include <type_traits>
#include <typeinfo>
#include <string>
#include <sstream>

namespace trajectory {
  template<class S>
  class TimedState : public geometry::State<TimedState<S>> {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");

   protected:
    S state_;
    units::QTime t_; // time at state_
    units::QSpeed velocity_;
    units::QAcceleration acceleration_;

   public:
    TimedState() {
      state_ = S();
      t_ = 0;
      velocity_ = 0;
      acceleration_ = 0;
    }
    TimedState(TimedState<S>* state)  {
      state_ = state->state();
      t_ = state->t();
      velocity_ = state->velocity();
      acceleration_ = state->acceleration();
    }
    TimedState(S state) {
      state_ = state;
      t_ = 0;
      velocity_ = 0;
      acceleration_ = 0;
    }
    TimedState(S state, units::QTime t, units::QSpeed velocity, units::QAcceleration acceleration)  {
      state_ = state;
      t_ = t;
      velocity_ = velocity;
      acceleration_ = acceleration;
    }
    S state() {
      return state_;
    }
    void set_t(units::QTime t) {
      t_ = t;
    }
    units::QTime t() {
      return t_;
    }
    void set_velocity(units::QSpeed v)  {
      velocity_ = v;
    }
    units::QSpeed velocity() {
      return velocity_;
    }
    void set_acceleration(units::QAcceleration a)  {
      acceleration_ = a;
    }
    units::QAcceleration acceleration()  {
      return acceleration_;
    }
    std::string toCSV()  {
      std::ostringstream stringStream;
      stringStream << "TimedState, {" << state_.toCSV() << "}, " << t_.to_string() << ", ";
      stringStream << velocity_.to_string() << ", " << acceleration_.to_string();
      return stringStream.str();
    }
    std::string toString() {
      return toCSV();
    }
    TimedState<S> interpolate(TimedState<S> other, units::Number x)  {
      units::QTime new_t = INTERPOLATE(t(), other.t(), x);
      units::QTime delta_t = new_t - t();

      if (delta_t < 0.0 * units::second)
        return other.interpolate(this, 1.0 - x);

      bool reversing = velocity() < 0.0*units::mps || (FEQUALS(velocity(), 0.0*units::mps) && acceleration() < 0.0*units::mps2);
      units::QSpeed new_v = velocity() + acceleration() * delta_t;
      units::QLength new_s = (reversing ? -1.0 : 1.0) * (velocity() * delta_t + .5 * acceleration() * delta_t * delta_t);

      return TimedState<S>(state().interpolate(other.state(), new_s / state().distance(other.state())),
                           new_t,
                           new_v,
                           acceleration());
    }
    units::QLength distance(TimedState<S> other)  {
      return state_.distance(other.state());
    }
    bool operator==(TimedState other)  {
      return state_ == other.state();
    }

  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMEDSTATE_HPP_
