//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_DISTANCEVIEW_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_DISTANCEVIEW_HPP_

#include <type_traits>
#include <vector>
#include "../geometry/interfaces/State.hpp"
#include "timing/TimedState.hpp"
#include "TrajectoryView.hpp"
#include "Trajectory.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"

namespace trajectory {

  template <class S>
  class DistanceView : public TrajectoryView<S> {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");
    protected:
      Trajectory<S>* trajectory_;
      std::vector<units::QLength> distances_;
    public:
      DistanceView(Trajectory<S>* trajectory) {
        trajectory_ = trajectory;
        distances_.resize(trajectory->length(), 0.0);
        distances_[0] = 0.0;
        for (int i = 1; i < trajectory_->length(); ++i) {
          distances_[i] = distances_[i - 1] + trajectory_->getState(i - 1).distance(trajectory_->getState(i));
        }
      }
      /* ~DistanceView() {
        delete trajectory_;
      } */
      TrajectorySamplePoint<S> sample(double dist) {
        units::QLength distance(dist);
        if (distance >= last_interpolant()*units::metre)
          return TrajectorySamplePoint<S>(trajectory_->getPoint(trajectory_->length() - 1));
        if (distance <= 0.0 * units::metre)
          return TrajectorySamplePoint<S>(trajectory_->getPoint(0));
        for (int i = 1; i < trajectory_->length(); ++i) {
          TrajectoryPoint<S> s = trajectory_->getPoint(i);
          if (distances_[i] >= distance) {
            TrajectoryPoint<S> prev_s = trajectory_->getPoint(i - 1);
            if (FEQUALS(distances_[i], distances_[i - 1])) {
              return TrajectorySamplePoint<S>(s);
            } else {
              return TrajectorySamplePoint<S>(prev_s.state().interpolate(s.state(),
                                                                         (distance - distances_[i - 1]) / (distances_[i] - distances_[i - 1])), i - 1, i);
            }
          }
        }
        return TrajectorySamplePoint<S>(S(), 0, 0); // THIS SHOULDNT HAPPEN
      }
      double last_interpolant() {
        return distances_.at(trajectory_->length() - 1).getValue();
      }
      double first_interpolant()  {
        return 0.0;
      }
      Trajectory<S>* trajectory() {
        return trajectory_;
      }

  };

}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_DISTANCEVIEW_HPP_
