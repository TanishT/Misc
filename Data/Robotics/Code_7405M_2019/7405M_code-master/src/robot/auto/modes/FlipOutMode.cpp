//
// Created by alexweiss on 8/14/19.
//

#include "FlipOutMode.hpp"
#include "../actions/DriveTrajectory.hpp"
#include "../actions/WaitAction.hpp"
#include "../actions/OpenLoopDriveAction.hpp"
#include "../actions/OpenLoopIntakeAction.hpp"
#include "../actions/ParallelAction.hpp"
#include "../actions/SeriesAction.hpp"
#include "../actions/TrayEnableStackAction.hpp"
#include "../../paths/TrajectorySet.hpp"
#include "../actions/TrayPosition.hpp"
#include "../actions/OpenLoopLiftAction.hpp"
#include "../actions/DriveTurnAction.hpp"
#include "../actions/ResetLiftTrayPosition.hpp"

namespace auton {
  void FlipOutMode::routine() {
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(200.0, 200.0), 1));
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(-200.0, -200.0), 1));

    flipOut();
  }

}
