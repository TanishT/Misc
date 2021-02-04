//
// Created by alexweiss on 8/4/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_INDEXVIEW_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_INDEXVIEW_HPP_

#include <memory>
#include "Trajectory.hpp"
#include "TrajectorySamplePoint.hpp"
#include "TrajectoryView.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"

namespace trajectory {
  template<class T>
  class IndexView : public TrajectoryView<T> {
   public:
    Trajectory<T>* pointer_to_;
    IndexView(Trajectory<T>* x) : pointer_to_(x) {}
    /* ~IndexView() {
      delete pointer_to_;
    } */
    TrajectorySamplePoint<T> sample(double index)  {
      return pointer_to_->getInterpolated(index);
    }
    double last_interpolant()  {
      return pointer_to_->length() == 0 ? 0 : pointer_to_->length() - 1;
    }
    double first_interpolant()  {
      return 0.0;
    }
    Trajectory<T>* trajectory()  {
      return pointer_to_;
    }
  };


}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_INDEXVIEW_HPP_
