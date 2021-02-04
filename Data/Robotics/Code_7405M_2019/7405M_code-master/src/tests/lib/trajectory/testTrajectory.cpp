//
// Created by alexweiss on 8/6/19.
//

#include "testTrajectory.hpp"
#include "../../../lib/trajectory/TrajectoryIterator.hpp"
#include "../../../lib/trajectory/Trajectory.hpp"
#include "../../../lib/trajectory/IndexView.hpp"
#include "../../../lib/trajectory/DistanceView.hpp"
#include "../../../lib/trajectory/TimedView.hpp"
#include "../../../lib/trajectory/IPathFollower.hpp"
#include "../../../lib/trajectory/PurePursuitController.hpp"
#include "../../../lib/trajectory/TrajectoryUtil.hpp"
#include "../../../lib/trajectory/timing/DifferentialDriveDynamicsConstraint.hpp"
#include "../../../lib/trajectory/timing/TimingUtil.hpp"
#include "../../../lib/physics/DCMotorTransmission.hpp"
#include "../../../lib/physics/DifferentialDrive.hpp"
#include "../../../lib/geometry/Translation2d.hpp"
#include "../../../lib/utility/Units.hpp"
#include <vector>
#include <memory>
#include <stdio.h>
#include <iostream>

namespace test {
  void testTrajectory::testTrajectoryClass() {
    std::vector<geometry::Translation2d> kWaypoints;
    kWaypoints.push_back(geometry::Translation2d(0.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(24.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(36.0, 12.0));
    kWaypoints.push_back(geometry::Translation2d(60.0, 12.0));

    trajectory::Trajectory<geometry::Translation2d> traj;
    std::shared_ptr<trajectory::IndexView<geometry::Translation2d>> idxView = traj.createIndexView();

    assertTrue(traj.isEmpty());
    assertEquals(0.0, idxView->first_interpolant(), EPSILON);
    assertEquals(0.0, idxView->last_interpolant(), EPSILON);
    assertEquals(0, traj.length());

    // Set states at construction time.

    traj = trajectory::Trajectory<geometry::Translation2d>(kWaypoints);
    assertFalse(traj.isEmpty());
    assertEquals(0.0, idxView->first_interpolant(), EPSILON);
    assertEquals(3.0, idxView->last_interpolant(), EPSILON);
    assertEquals(4, traj.length());

    assertEquals(kWaypoints.at(0), traj.getState(0));
    assertEquals(kWaypoints.at(1), traj.getState(1));
    assertEquals(kWaypoints.at(2), traj.getState(2));
    assertEquals(kWaypoints.at(3), traj.getState(3));

    assertEquals(kWaypoints.at(0), traj.getInterpolated(0.0).state());
    assertEquals(traj.getInterpolated(0.0).index_floor(), 0);
    assertEquals(traj.getInterpolated(0.0).index_ceil(), 0);
    assertEquals(kWaypoints.at(1), traj.getInterpolated(1.0).state());
    assertEquals(traj.getInterpolated(1.0).index_floor(), 1);
    assertEquals(traj.getInterpolated(1.0).index_ceil(), 1);
    assertEquals(kWaypoints.at(2), traj.getInterpolated(2.0).state());
    assertEquals(traj.getInterpolated(2.0).index_floor(), 2);
    assertEquals(traj.getInterpolated(2.0).index_ceil(), 2);
    assertEquals(kWaypoints.at(3), traj.getInterpolated(3.0).state());
    assertEquals(traj.getInterpolated(3.0).index_floor(), 3);
    assertEquals(traj.getInterpolated(3.0).index_ceil(), 3);

    assertEquals(kWaypoints.at(0).interpolate(kWaypoints.at(1), .25), traj.getInterpolated(0.25).state());
    assertEquals(traj.getInterpolated(0.25).index_floor(), 0);
    assertEquals(traj.getInterpolated(0.25).index_ceil(), 1);
    assertEquals(kWaypoints.at(1).interpolate(kWaypoints.at(2), .5), traj.getInterpolated(1.5).state());
    assertEquals(traj.getInterpolated(1.5).index_floor(), 1);
    assertEquals(traj.getInterpolated(1.5).index_ceil(), 2);
    assertEquals(kWaypoints.at(2).interpolate(kWaypoints.at(3), .75), traj.getInterpolated(2.75).state());
    assertEquals(traj.getInterpolated(2.75).index_floor(), 2);
    assertEquals(traj.getInterpolated(2.75).index_ceil(), 3);

    assertEquals(kWaypoints.at(0).interpolate(kWaypoints.at(1), .25), idxView->sample(0.25).state());
    assertEquals(kWaypoints.at(1).interpolate(kWaypoints.at(2), .5), idxView->sample(1.5).state());
    assertEquals(kWaypoints.at(2).interpolate(kWaypoints.at(3), .75), idxView->sample(2.75).state());

  }

  void testTrajectory::testTrajectoryIterator() {
    std::vector<geometry::Translation2d> kWaypoints;
    kWaypoints.push_back(geometry::Translation2d(0.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(24.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(36.0, 12.0));
    kWaypoints.push_back(geometry::Translation2d(60.0, 12.0));

    trajectory::Trajectory<geometry::Translation2d> traj(kWaypoints);

    std::shared_ptr<trajectory::TrajectoryView<geometry::Translation2d>> indexView = traj.createIndexView();
    trajectory::TrajectoryIterator<geometry::Translation2d> iterator(indexView);
    iterator.setup();

    // Initial conditions.
    assertEquals(0.0, iterator.getProgress(), EPSILON);
    assertEquals(3.0, iterator.getRemainingProgress(), EPSILON);
    assertEquals(kWaypoints.at(0), iterator.getState());
    assertFalse(iterator.isDone());

    // Advance forward.
    assertEquals(kWaypoints.at(0).interpolate(kWaypoints.at(1), 0.5), iterator.preview(0.5).state());
    assertEquals(kWaypoints.at(0).interpolate(kWaypoints.at(1), 0.5), iterator.advance(0.5).state());
    assertEquals(0.5, iterator.getProgress(), EPSILON);
    assertEquals(2.5, iterator.getRemainingProgress(), EPSILON);
    assertFalse(iterator.isDone());

    // Advance backwards.
    assertEquals(kWaypoints.at(0).interpolate(kWaypoints.at(1), 0.25), iterator.preview(-0.25).state());
    assertEquals(kWaypoints.at(0).interpolate(kWaypoints.at(1), 0.25), iterator.advance(-0.25).state());
    assertEquals(0.25, iterator.getProgress(), EPSILON);
    assertEquals(2.75, iterator.getRemainingProgress(), EPSILON);
    assertFalse(iterator.isDone());

    // Advance past end.
    assertEquals(kWaypoints.at(3), iterator.preview(5.0).state());
    assertEquals(kWaypoints.at(3), iterator.advance(5.0).state());
    assertEquals(3.0, iterator.getProgress(), EPSILON);
    assertEquals(0.0, iterator.getRemainingProgress(), EPSILON);
    assertTrue(iterator.isDone());

    // Advance past beginning.
    assertEquals(kWaypoints.at(0), iterator.preview(-5.0).state());
    assertEquals(kWaypoints.at(0), iterator.advance(-5.0).state());
    assertEquals(0.0, iterator.getProgress(), EPSILON);
    assertEquals(3.0, iterator.getRemainingProgress(), EPSILON);
    assertFalse(iterator.isDone());
  }

  void testTrajectory::testDistanceView() {
    std::vector<geometry::Translation2d> kWaypoints;
    kWaypoints.push_back(geometry::Translation2d(0.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(24.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(36.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(36.0, 24.0));
    kWaypoints.push_back(geometry::Translation2d(60.0, 24.0));

    // Create the reference trajectory (straight line motion  between waypoints).
    trajectory::Trajectory<geometry::Translation2d> traj(kWaypoints);
    trajectory::DistanceView<geometry::Translation2d> distance_view (&traj);

    assertEquals(0.0, distance_view.first_interpolant(), EPSILON);
    assertEquals(84.0, distance_view.last_interpolant(), EPSILON);

    assertEquals(kWaypoints.at(0), distance_view.sample(0.0).state());
    assertEquals(kWaypoints.at(0).interpolate(kWaypoints.at(1), 0.5),   distance_view.sample(12.0).state());
    assertEquals(kWaypoints.at(3).interpolate(kWaypoints.at(4), 0.5), distance_view.sample(72.0).state());
  }

  void testTrajectory::testIntegration() {
    std::vector<geometry::Translation2d> kWaypoints;
    kWaypoints.push_back(geometry::Translation2d(0.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(24.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(36.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(36.0, 24.0));
    kWaypoints.push_back(geometry::Translation2d(60.0, 24.0));

    trajectory::Trajectory<geometry::Translation2d> reference_trajectory(kWaypoints);
    trajectory::DistanceView<geometry::Translation2d> distance_view(&reference_trajectory);

    // Generate a smooth (continuous curvature) path to follow.

    trajectory::IPathFollower *path_follower =
        new trajectory::PurePursuitController<geometry::Translation2d>(&distance_view,  /* sampling_dist */
                                                                       .25, /* lookahead= */
                                                                       2.0,  /* goal_tolerance= */
                                                                       0.1);
    trajectory::Trajectory<geometry::Pose2dWithCurvature> smooth_path =
        trajectory::TrajectoryUtil::trajectoryFromPathFollower(path_follower,
            geometry::Pose2dWithCurvature(),
                                                               .25,
                                                               1.0);
    assertFalse(smooth_path.isEmpty());

    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0.0, 0.0, geometry::Rotation2d::fromDegrees(0.0)));
    waypoints.push_back(geometry::Pose2d(3.6, 0.0, geometry::Rotation2d::fromDegrees(0.0)));
    waypoints.push_back(geometry::Pose2d(6.0, 10.0, geometry::Rotation2d::fromDegrees(0.0)));
    waypoints.push_back(geometry::Pose2d(16.0, 10.0, geometry::Rotation2d::fromDegrees(0.0)));
    waypoints.push_back(geometry::Pose2d(20.0, 7.0, geometry::Rotation2d::fromDegrees(45.0)));

    // Create a trajectory from splines.
    trajectory::Trajectory<geometry::Pose2dWithCurvature>
        traj = trajectory::TrajectoryUtil::trajectoryFromSplineWaypoints(waypoints,
                                                                         2.0,
                                                                         0.2,
                                                                          geometry::Rotation2d::fromDegrees(5.0).getRadians());
    // System.out.println(trajectory.toCSV());
    // Create a differential drive.

    units::QMass kRobotMassKg = 60.0;
    units::QMoment kRobotAngularInertia = 80.0;
    units::QAngularDrag kRobotAngularDrag = 0.0;
    units::QLength kWheelRadius = (2.0 * units::inch);
    units::QLength kWheelbaseRadius =  (26.0 / 2.0) * units::inch;
    printf("WHEEL RADIUS %f\n", kWheelRadius);
    printf("WHEELBASE RADIUS %f\n", kWheelbaseRadius);
    physics::DCMotorTransmission
        transmission(6.993, 3.87096, 0.8);
    physics::DifferentialDrive drive(kRobotMassKg,
                                     kRobotAngularInertia,
                                     kRobotAngularDrag,
                                     kWheelRadius,
                                     kWheelbaseRadius,
                                     transmission,
                                     transmission);
    // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
    // than 10V.
    trajectory::DifferentialDriveDynamicsConstraint<geometry::Pose2dWithCurvature> drive_constraints(&drive, 10.0);
    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature> *> constraints_list;
    constraints_list.push_back(&drive_constraints);
    // Generate the timed trajectory.

    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>>
        timed_trajectory = trajectory::TimingUtil::timeParameterizeTrajectory(
        false, trajectory::DistanceView<geometry::Pose2dWithCurvature>(&traj), .05, constraints_list,
        0.0, 0.0, 12.0 * 14.0, 12.0 * 10.0);





    for(int i = 0; i < timed_trajectory.length(); i++) {
      printf("(%f, %f)\n", timed_trajectory.getState(i).t(), timed_trajectory.getState(i).state().translation().x());
    }

    for (int i = 1; i < timed_trajectory.length(); ++i) {
      trajectory::TrajectoryPoint<trajectory::TimedState<geometry::Pose2dWithCurvature>>
          prev = timed_trajectory.getPoint(i - 1);
      trajectory::TrajectoryPoint<trajectory::TimedState<geometry::Pose2dWithCurvature>>
          next = timed_trajectory.getPoint(i);
      assertEquals(prev.state().acceleration(), (next.state().velocity() - prev.state().velocity()) / (next
          .state().t() - prev.state().t()), 1E-9);
      units::QTime dt = next.state().t() - prev.state().t();
      assertEquals(next.state().velocity(), prev.state().velocity() + prev.state().acceleration() * dt, 1E-9);
      assertEquals(next.state().distance(prev.state()), prev.state().velocity() * dt + 0.5 * prev.state()
          .acceleration() * dt * dt, 1E-9);
    }

    // "Follow" the trajectory.
    double kDt = 0.05;
    bool first = true;
    std::shared_ptr<trajectory::TimedView<geometry::Pose2dWithCurvature>>
        ptr_to_timed_view = std::make_shared<trajectory::TimedView<geometry::Pose2dWithCurvature>>(&timed_trajectory);
    trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> it(ptr_to_timed_view);
    while (!it.isDone()) {
      trajectory::TrajectorySamplePoint<trajectory::TimedState<geometry::Pose2dWithCurvature>>
          sample(trajectory::TimedState<geometry::Pose2dWithCurvature>(), 0, 0);
      if (first) {
        sample = it.getSample();
        first = false;
      } else {
        sample = it.advance(kDt);
      }
      trajectory::TimedState<geometry::Pose2dWithCurvature> state = sample.state();

      physics::DifferentialDrive::DriveDynamics dynamics = drive.solveInverseDynamics(
          physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(state.velocity(),
                                                                                        state.velocity()
                                                                                            * state.state().curvature()),
          physics::DifferentialDrive::ChassisState<units::QAcceleration,
                                                   units::QAngularAcceleration>((state.acceleration()),
                                                                                state.acceleration()
                                                                                    * state.state().curvature()));

      std::cout << "(" << state.t() << "," << dynamics.wheel_velocity.left_ << ")" << std::endl;
    }
  }

    void testTrajectory::realisticConstants() {
      std::vector<geometry::Pose2d> waypoints;
      /*waypoints.push_back(geometry::Pose2d(0.0, 0.0, geometry::Rotation2d::fromDegrees(0.0)));
      waypoints.push_back(geometry::Pose2d(1.0, 0.0, geometry::Rotation2d::fromDegrees(0.0)));
      waypoints.push_back(geometry::Pose2d(3.0, 2.0, geometry::Rotation2d::fromDegrees(0.0)));
      waypoints.push_back(geometry::Pose2d(4.0, 2.0, geometry::Rotation2d::fromDegrees(45.0)));*/

      waypoints.push_back(geometry::Pose2d(0.0, 0.0, geometry::Rotation2d::fromDegrees(90.0)));
      waypoints.push_back(geometry::Pose2d(0.5, 0.25, geometry::Rotation2d::fromDegrees(0)));
      waypoints.push_back(geometry::Pose2d(1.0, 0.0, geometry::Rotation2d::fromDegrees(270.0)));

      trajectory::Trajectory<geometry::Pose2dWithCurvature>
          traj = trajectory::TrajectoryUtil::trajectoryFromSplineWaypoints(waypoints,
                                                                           2.0,
                                                                           0.2,
                                                                           geometry::Rotation2d::fromDegrees(5.0).getRadians());
      // System.out.println(trajectory.toCSV());
      // Create a differential drive.

      units::QMass kRobotMassKg = 8;
      units::QMoment kRobotAngularInertia = 12;
      units::QAngularDrag kRobotAngularDrag = 0.0;
      units::QLength kWheelRadius = (2.0 * units::inch);
      units::QLength kWheelbaseRadius =  (18.0 / 2.0) * units::inch;
      printf("WHEEL RADIUS %f\n", kWheelRadius);
      printf("WHEELBASE RADIUS %f\n", kWheelbaseRadius);
      physics::DCMotorTransmission
          transmission(1.903, .175, 1);
      physics::DifferentialDrive drive(kRobotMassKg,
                                       kRobotAngularInertia,
                                       kRobotAngularDrag,
                                       kWheelRadius,
                                       kWheelbaseRadius,
                                       transmission,
                                       transmission);
      // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
      // than 10V.
      trajectory::DifferentialDriveDynamicsConstraint<geometry::Pose2dWithCurvature> drive_constraints(&drive, 12);
      std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature> *> constraints_list;
      constraints_list.push_back(&drive_constraints);
      // Generate the timed trajectory.

      trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>>
          timed_trajectory = trajectory::TimingUtil::timeParameterizeTrajectory(
          true, trajectory::DistanceView<geometry::Pose2dWithCurvature>(&traj), .05, constraints_list,
          0.0, 0.0, 12.0 * 14.0, 12.0 * 10.0);


      /* for(int i = 0; i < timed_trajectory.length(); i++) {
        printf("(%f, %f)\n", timed_trajectory.getState(i).state().translation().x(), timed_trajectory.getState(i).state().translation().y());
      } */

      double kDt = 0.05;
      std::shared_ptr<trajectory::TimedView<geometry::Pose2dWithCurvature>> ptr_to_timed_view = std::make_shared<trajectory::TimedView<geometry::Pose2dWithCurvature>>(&timed_trajectory);
      trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> it(ptr_to_timed_view);
      while (!it.isDone()) {
        trajectory::TrajectorySamplePoint<trajectory::TimedState<geometry::Pose2dWithCurvature>>
            sample(trajectory::TimedState<geometry::Pose2dWithCurvature>(), 0, 0);
        sample = it.advance(kDt);
        trajectory::TimedState<geometry::Pose2dWithCurvature> state = sample.state();

        physics::DifferentialDrive::DriveDynamics dynamics = drive.solveInverseDynamics(
            physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(state.velocity(), state.velocity() * state.state().curvature()),
            physics::DifferentialDrive::ChassisState<units::QAcceleration, units::QAngularAcceleration>((state.acceleration()), state.acceleration() * state.state().curvature()));

        //printf("(%f, %f)\n", state.t(), dynamics.wheel_velocity.left_);
        printf("(%f, %f)\n", state.t(), dynamics.chassis_velocity.linear_);

      }
    }
    void testTrajectory::newTest() {
      printf("started");
      std::vector<geometry::Pose2d> waypoints;
      waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
      waypoints.push_back(geometry::Pose2d(10 * units::inch, 10 * units::inch, geometry::Rotation2d::fromDegrees(0)));
      waypoints.push_back(geometry::Pose2d(0 * units::inch, 20 * units::inch, geometry::Rotation2d::fromDegrees(180)));
      waypoints.push_back(geometry::Pose2d(-10 * units::inch, 30 * units::inch, geometry::Rotation2d::fromDegrees(90)));
      waypoints.push_back(geometry::Pose2d(0 * units::inch, 40 * units::inch, geometry::Rotation2d::fromDegrees(0)));

      trajectory::Trajectory<geometry::Pose2dWithCurvature>
          traj = trajectory::TrajectoryUtil::trajectoryFromSplineWaypoints(waypoints,
                                                                           0.2,
                                                                           0.05,
                                                                           geometry::Rotation2d::fromDegrees(3.0).getRadians());


      std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature> *> constraints_list;
      printf("stcuk");



      trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>>
          timed_trajectory = trajectory::TimingUtil::timeParameterizeTrajectory(
          false, trajectory::DistanceView<geometry::Pose2dWithCurvature>(&traj), .05, constraints_list,
          0, 0, 1, .33);

      for(int i = 0; i < traj.length(); i++)
        std::cout << (traj.getState(i).toString()) << std::endl;


    }
}
