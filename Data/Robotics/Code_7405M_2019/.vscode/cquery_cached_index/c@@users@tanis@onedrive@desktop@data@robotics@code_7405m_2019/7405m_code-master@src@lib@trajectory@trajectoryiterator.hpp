//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYITERATOR_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYITERATOR_HPP_

#include <type_traits>
#include <memory>
#include "../geometry/interfaces/State.hpp"
#include "Trajectory.hpp"
#include "TrajectoryView.hpp"
#include "TrajectorySamplePoint.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"
#include "timing/TimedState.hpp"
#include "../utility/Utility.hpp"


namespace trajectory {

  template <class S>
  class TrajectoryIterator {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");
    protected:
      std::shared_ptr<TrajectoryView<S>> view_;
      double progress_;
      TrajectorySamplePoint<S> current_sample_;

    public:
      TrajectoryIterator(std::shared_ptr<TrajectoryView<S>> view)  :
          current_sample_(S(), 0, 0), progress_(0.0)
      {
        view_ = view;
      }

      void setup() {
        current_sample_ = view_->sample(view_->first_interpolant());
        progress_ = view_->first_interpolant();
      }

      bool isDone() {
        return getRemainingProgress() == 0.0;
      }
      double getProgress() {
        return progress_;
      }
      double getRemainingProgress() {
        return MAX(0.0, view_->last_interpolant() - progress_);
      }
      TrajectorySamplePoint<S> getSample() {
        return current_sample_;
      }
      S getState()  {
        return getSample().state();
      }
      TrajectorySamplePoint<S> advance(double additional_progress)  {
        progress_ = MAX(view_->first_interpolant(),
                        MIN(view_->last_interpolant(), progress_ + additional_progress));  current_sample_ = view_->sample(progress_);
        return current_sample_;
      }
      TrajectorySamplePoint<S> preview(double additional_progress)  {
        double progress = MAX(view_->first_interpolant(),
                              MIN(view_->last_interpolant(), progress_ + additional_progress));
        return view_->sample(progress);
      }
      Trajectory<S>* trajectory() {
        return view_->trajectory();
      }
  };



}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYITERATOR_HPP_
