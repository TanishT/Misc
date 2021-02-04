//
// Created by alexweiss on 8/15/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_DRIVETRAJECTORY_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_DRIVETRAJECTORY_HPP_

#include "Action.hpp"
#include "../../../lib/meecan_lib.hpp"
#include <string>

namespace auton {
  namespace actions {
    class DriveTrajectory : public Action {
     private:
      trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> trajectory_;
      bool reset_pose_;
     public:
      DriveTrajectory(std::string name);
      DriveTrajectory(std::string name, bool reset_pose);
      DriveTrajectory(trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> trajectory);
      DriveTrajectory(trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> trajectory, bool reset_pose);
      bool isFinished();
      void start();
      void update();
      void done();



    };
  }
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_DRIVETRAJECTORY_HPP_
