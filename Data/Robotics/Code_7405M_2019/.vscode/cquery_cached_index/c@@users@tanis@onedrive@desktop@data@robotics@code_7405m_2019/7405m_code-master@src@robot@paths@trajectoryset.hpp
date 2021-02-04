//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_PATHS_TRAJECTORYSET_HPP_
#define INC_7405M_CODE_SRC_ROBOT_PATHS_TRAJECTORYSET_HPP_

#include "../../lib/meecan_lib.hpp"

namespace path_planning {

  class MirroredTrajectory {
    public:
      trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> backward_;
      trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> forward_;

      MirroredTrajectory(trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> right);
      trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> get(bool left);
  };

  class TrajectorySet {
    private:
      std::map<std::string, MirroredTrajectory> trajectories_;
      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> backJForward();
      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> backLineForward();
      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> backToLine();
      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> backBackForTowerCube();
      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> backGetTowerCube();
      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> backAlign();
      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> backSetup();

      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> pSkillsIntake();
      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> pSkillsSetup();

      /* static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getFrontForward();
      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getFrontIntake();
      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getFrontS();
      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getFrontSetup(); */

      static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> stackPullBack();




    public:
      bool complete_;

      TrajectorySet();
      void startGenerator();
      bool isDoneGenerating();
      bool inSet(std::string name);
      MirroredTrajectory get(std::string name);
      void addToMap(std::string name, MirroredTrajectory traj);
      void generatorCalls();
      struct TrajectorySetManager : public util::Singleton<TrajectorySet, TrajectorySetManager> {};
      static TrajectorySetManager instance;
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_PATHS_TRAJECTORYSET_HPP_
