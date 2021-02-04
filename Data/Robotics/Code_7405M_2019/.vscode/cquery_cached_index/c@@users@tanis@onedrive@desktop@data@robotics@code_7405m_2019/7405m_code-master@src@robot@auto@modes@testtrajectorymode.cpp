//
// Created by alexweiss on 10/20/19.
//

#include "TestTrajectoryMode.hpp"
#include "../actions/DriveTrajectory.hpp"
#include "../../paths/TrajectorySet.hpp"

namespace auton {
  void TestTrajectoryMode::routine() {
    printf("started test mode\n");
    runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pptest").get(false)));
  }

}
