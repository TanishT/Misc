//
// Created by alexweiss on 8/6/19.
//

#include <stdio.h>
#include <iostream>
#include "testTimedState.hpp"
#include "../../../../lib/geometry/Pose2d.hpp"
#include "../../../../lib/geometry/Translation2d.hpp"
#include "../../../../lib/trajectory/timing/TimedState.hpp"


namespace test {
  void testTimedState::test() {
    trajectory::TimedState<geometry::Pose2d> start_state(geometry::Pose2d::fromTranslation(geometry::Translation2d(0.0, 0.0)),
                                                      0.0, 0.0, 1.0);

    // At (.5,0,0), t=1, v=1, acceleration=0
    trajectory::TimedState<geometry::Pose2d> end_state(geometry::Pose2d::fromTranslation(geometry::Translation2d(0.5, 0.0)), 1.0,
                                                          1.0, 0.0);

    assertEquals(start_state, start_state.interpolate(end_state, 0.0));

    assertEquals(end_state, start_state.interpolate(end_state, 1.0));
    assertEquals(end_state, end_state.interpolate(start_state, 0.0));
    //System.out.println(end_state.interpolate(start_state, 1.0));
    assertEquals(start_state, end_state.interpolate(start_state, 1.0));

    trajectory::TimedState<geometry::Pose2d> intermediate_state = start_state.interpolate(end_state, 0.5);
    assertEquals(0.5, intermediate_state.t(), EPSILON);
    assertEquals(start_state.acceleration(), intermediate_state.acceleration(), EPSILON);
    assertEquals(0.5, intermediate_state.velocity(), EPSILON);
    assertEquals(0.125, intermediate_state.state().translation().x().getValue(), EPSILON);
  }
}