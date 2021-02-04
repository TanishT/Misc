//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_PUREPURSUITCONTROLLER_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_PUREPURSUITCONTROLLER_HPP_

#include <type_traits>
#include "../geometry/interfaces/State.hpp"
#include "../geometry/interfaces/ITranslation2d.hpp"
#include "../geometry/Twist2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Translation2d.hpp"
#include "IPathFollower.hpp"
#include "TrajectoryIterator.hpp"
#include "DistanceView.hpp"

namespace trajectory {

  template <class S>
  class Arc {
    static_assert(std::is_base_of<geometry::ITranslation2d<S>, S>::value, "T must derive from State");
   protected:
    geometry::Translation2d findCenter(geometry::Pose2d pose, S point)  {
      geometry::Translation2d poseToPointHalfway = pose.translation().interpolate(point.translation(), 0.5);
      geometry::Rotation2d normal = pose.translation().inverse().translateBy(poseToPointHalfway).direction()
          .normal();
      geometry::Pose2d perpendicularBisector = geometry::Pose2d(poseToPointHalfway, normal);
      geometry::Pose2d normalFromPose = geometry::Pose2d(pose.translation(),
                                                         pose.rotation().normal());
      if (normalFromPose.IsLinear(perpendicularBisector.normal())) {
        // Special case: center is poseToPointHalfway.
        return poseToPointHalfway;
      }
      return normalFromPose.intersection(perpendicularBisector);
    }

    units::QLength findLength(geometry::Pose2d pose, S point, geometry::Translation2d center, units::QLength radius)  {
      if(radius.getValue() == INFINITY) {
        return pose.translation().distance(point.translation());
      }
      geometry::Translation2d centerToPoint = geometry::Translation2d(center, point.translation());
      geometry::Translation2d centerToPose = geometry::Translation2d(center, pose.translation());
      // If the point is behind pose, we want the opposite of this angle. To determine if the point is behind,
      // check the sign of the cross-product between the normal vector and the vector from pose to point.
      bool behind = util::sgn(
          geometry::Translation2d::cross(pose.rotation().normal().toTranslation(),
                                         geometry::Translation2d(pose.translation(), point.translation()))) > 0.0;
      geometry::Rotation2d angle = geometry::Translation2d::getAngle(centerToPose, centerToPoint);
      return radius * (behind ? 2.0 * M_PI - std::fabs(angle.getRadians()) : std::fabs(angle.getRadians()));
    }

    template <class T> static double getDirection(geometry::Pose2d pose, T point) {
      geometry::Translation2d poseToPoint = geometry::Translation2d(pose.translation(), point.translation());
      geometry::Translation2d robot = pose.rotation().toTranslation();
      double cross = (robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x()).getValue();
      return (cross < 0.) ? -1. : 1.; // if robot < pose turn left
    }
   public:
    geometry::Translation2d center;
    units::QLength radius;
    units::QLength length;
    Arc(geometry::Pose2d pose, S point) {
      center = findCenter(pose, point);
      radius = geometry::Translation2d(center, point.translation()).norm().getValue();
      length = findLength(pose, point, center, radius);
      radius *= getDirection(pose, point);
    }
  };

  template <class S>
  class PurePursuitController : public IPathFollower {
    static_assert(std::is_base_of<geometry::ITranslation2d<S>, S>::value, "S must derive from State");
    protected:
      TrajectoryIterator<S>* iterator_;
      units::QLength sampling_dist_;
      units::QLength lookahead_;
      units::QLength goal_tolerance_;
      bool done_ = false;

    public:
      PurePursuitController(DistanceView<S>* path, units::QLength sampling_dist, units::QLength lookahead, units::QLength goal_tolerance) {
        sampling_dist_ = sampling_dist;
        lookahead_ = lookahead;
        goal_tolerance_ = goal_tolerance;
        std::shared_ptr<TrajectoryView<S>> path_ptr(path);
        iterator_ = new TrajectoryIterator<S>(path_ptr);
      }
      /* ~PurePursuitController() {
        delete iterator_;
      } */
      geometry::Twist2d steer(geometry::Pose2d current_pose)  {
        done_ = done_ || (iterator_->isDone()
            && current_pose.translation().distance(iterator_->getState().translation()) <= goal_tolerance_);
        if (isDone()) {
          return geometry::Twist2d(0.0, 0.0, 0.0);
        }

        units::QLength remaining_progress = iterator_->getRemainingProgress();
        units::QLength goal_progress = 0.0;
        // Find the first point > lookahead distance away from current_pose, or the last point otherwise.
        for (units::QLength progress = 0.0; progress <= remaining_progress; progress = MIN(remaining_progress,
                                                                                   progress + sampling_dist_)) {

          units::QLength dist = current_pose.translation().distance(iterator_->preview(progress.getValue()).state().translation());
          if (dist > lookahead_) {
            if (goal_progress == 0.0 * units::metre && !iterator_->isDone()) {
              // Make sure we don't get stuck due to numerical issues when sampling dist is large relative to
              // lookahead.
              goal_progress = progress;
            }
            break;
          }
          goal_progress = progress;
          if (progress == remaining_progress) {
            break;
          }
        }

        iterator_->advance(goal_progress.getValue());

        Arc<S> arc(current_pose, iterator_->getState());
        if (arc.length < EPSILON*units::metre) {
          return geometry::Twist2d(0.0, 0.0, 0.0);
        }
        else {
          return geometry::Twist2d(arc.length, 0.0, arc.length / arc.radius * units::radian);
        }
      }
      bool isDone() {
        return done_;
      }
  };

}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_PUREPURSUITCONTROLLER_HPP_
