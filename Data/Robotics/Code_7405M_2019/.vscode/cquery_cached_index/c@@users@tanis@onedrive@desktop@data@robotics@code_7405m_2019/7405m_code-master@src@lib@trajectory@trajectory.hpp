//
// Created by alexweiss on 8/2/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORY_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORY_HPP_

#include <type_traits>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include <memory>
#include "../geometry/interfaces/State.hpp"
#include "../utility/Utility.hpp"
#include "TrajectoryPoint.hpp"
#include "TrajectorySamplePoint.hpp"
#include "timing/TimedState.hpp"
#include "IndexView.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"

namespace trajectory {
  template<class T>   class IndexView;

  template<class S>
  class Trajectory {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S is not derived from State");

    protected:
      std::vector<TrajectoryPoint<S>> points_;
    public:
      Trajectory()  {
        points_ = std::vector<TrajectoryPoint<S>>();
      }
      Trajectory(std::vector<S> states)  {
        points_ = std::vector<TrajectoryPoint<S>>();
        for (int i = 0; i < states.size(); ++i) {
          points_.push_back(TrajectoryPoint<S>(states.at(i), i));
        }
      }
      int length() {
        return points_.size();
      }
      bool isEmpty() {
        return length() == 0;
      }
      TrajectoryPoint<S> getPoint(int index) {
        return points_.at(index);
      }
      S getState(int index) {
        return points_.at(index).state();
      }
      S getFirstState() {
        return points_.at(0).state();
      }
      S getLastState()  {
        return points_.at(length() - 1).state();
      }
      TrajectorySamplePoint<S> getInterpolated(units::Number index)  {
        if (isEmpty()) {
          return  TrajectorySamplePoint<S>(S(), 0, 0);
        } else if (index <= 0.0 * units::num) {
          return TrajectorySamplePoint<S>(getPoint(0));
        } else if (index >= (length() - 1)*units::num) {
          return TrajectorySamplePoint<S>(getPoint(length() - 1));
        }
        int i = (int) std::floor(index.getValue());
        units::Number frac = index - i;
        if (frac <= EPSILON * units::num) {
          return TrajectorySamplePoint<S>(getPoint(i));
        } else if (frac >= (1.0 - EPSILON)*units::num) {
          return TrajectorySamplePoint<S>(getPoint(i + 1));
        } else {
          return TrajectorySamplePoint<S>(getState(i).interpolate(getState(i + 1), frac), i, i + 1);
        }
      }
      std::shared_ptr<IndexView<S>> createIndexView() {
        return std::shared_ptr<IndexView<S>>(new IndexView<S>(this));
      }
      std::string toCSV() {
         return "Trajectory<>";
      }
      std::string toString() {
        return toCSV();
      }
  };
}
#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORY_HPP_
