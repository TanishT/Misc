//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMEDVIEW_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMEDVIEW_HPP_

#include <type_traits>
#include "../geometry/interfaces/State.hpp"
#include "timing/TimedState.hpp"
#include "Trajectory.hpp"
#include "TrajectoryView.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"

namespace trajectory {

  template <class S>
  class TimedView : public TrajectoryView<TimedState<S>> {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");
    protected:
      Trajectory<TimedState<S>>* trajectory_;
      units::QTime start_t_;
      units::QTime end_t_;
    public:
      TimedView(Trajectory<TimedState<S>>* trajectory)  {
        trajectory_ = trajectory;
        start_t_ = trajectory_->getState(0).t();
        end_t_ = trajectory_->getState(trajectory_->length() - 1).t();
      }
      /* ~TimedView() {
        delete trajectory_;
      } */
      double first_interpolant()  {
        return start_t_.getValue();
      }
      double last_interpolant() {
        return end_t_.getValue();
      }
      TrajectorySamplePoint<TimedState<S>> sample(double t)  {
        if (t >= end_t_.getValue()) {
          return TrajectorySamplePoint<TimedState<S>>(trajectory_->getPoint(trajectory_->length() - 1));
        }
        if (t <= start_t_.getValue()) {
          return TrajectorySamplePoint<TimedState<S>>(trajectory_->getPoint(0));
        }
        for (int i = 1; i < trajectory_->length(); ++i) {
          TrajectoryPoint<TimedState<S>> s = trajectory_->getPoint(i);
          if (s.state().t() >= t*units::second) {
            TrajectoryPoint<TimedState<S>> prev_s = trajectory_->getPoint(i - 1);
            if (FEQUALS(s.state().t(), prev_s.state().t())) {
              return TrajectorySamplePoint<TimedState<S>>(s);
            }
            return TrajectorySamplePoint<TimedState<S>>(prev_s.state().interpolate(s.state(),
                                                                                   (t*units::second - prev_s.state().t()) / (s.state().t() - prev_s.state().t())), i - 1, i);
          }
        }
        return TrajectorySamplePoint<TimedState<S>>(S(), 0, 0); // THIS SHOULDNT HAPPEN
      }
      Trajectory<TimedState<S>>* trajectory()  {
        return trajectory_;
      }

  };

}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMEDVIEW_HPP_
