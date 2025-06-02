/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#include <gtest/gtest.h>

#include <tmc_timeopt/trajectory_filter_via_stop_state.hpp>

#include "trajectory_filter_common-test.hpp"

namespace tmc_timeopt {

INSTANTIATE_TYPED_TEST_SUITE_P(TrajectoryFilterViaStopStateTest,
                               TrajectoryFilterCommonTest,
                               TrajectoryFilterViaStopState);

// Optimization from a stationary state
TEST(TrajectoryFilterViaStopStateTest, InitialVelocityZero) {
  auto input = TestInput();

  auto trajectory = TrajectoryFilterViaStopState(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  ASSERT_TRUE(trajectory.IsValid());

  EXPECT_TRUE(VerifyEndVelocity(trajectory));
  // Set kMaxDeviation to 0.03, so it should be within that range
  EXPECT_LT(CalcWayPointDistance(trajectory, input.way_points[0]), 0.03 * 2);
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[1]), kEpsilon);
  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, input.max_accelerations));
}

// Optimization from an operational state, initial velocity is just one positive value
TEST(TrajectoryFilterViaStopStateTest, OnePositiveInitialVelocity) {
  auto input = TestInput();
  input.initial_velocities << input.max_velocities[0], 0.0;

  auto trajectory = TrajectoryFilterViaStopState(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  ASSERT_TRUE(trajectory.IsValid());

  EXPECT_TRUE(VerifyEndVelocity(trajectory));
  EXPECT_LT(CalcWayPointDistance(trajectory, input.way_points[0]), 0.03 * 2);
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[1]), kEpsilon);
  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, input.max_accelerations));

  // Initial velocity is 1.0, maximum acceleration is 0.5, so it temporarily stops in 2 seconds
  EXPECT_TRUE(VerifyZeroVelocity(trajectory, 2.0, "stop state velocities"));
  Eigen::VectorXd stop_state_pos;
  stop_state_pos.resize(2);
  stop_state_pos << 1.0, 0.0;
  EXPECT_LT(CalcWayPointDistance(trajectory, stop_state_pos), kEpsilon);

  // Check an appropriate waypoint
  EXPECT_NEAR(trajectory.GetPosition(1.0)[0], 0.75, kEpsilon);
  EXPECT_NEAR(trajectory.GetPosition(1.0)[1], 0.0, kEpsilon);
  EXPECT_NEAR(trajectory.GetVelocity(1.0)[0], 0.5, kEpsilon);
  EXPECT_NEAR(trajectory.GetVelocity(1.0)[1], 0.0, kEpsilon);
}

// Optimization from an operational state, initial velocity is just one negative value
TEST(TrajectoryFilterViaStopStateTest, OneNegativeInitialVelocity) {
  auto input = TestInput();
  input.initial_velocities << -input.max_velocities[0], 0.0;

  auto trajectory = TrajectoryFilterViaStopState(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  ASSERT_TRUE(trajectory.IsValid());

  EXPECT_TRUE(VerifyEndVelocity(trajectory));
  EXPECT_LT(CalcWayPointDistance(trajectory, input.way_points[0]), 0.03 * 2);
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[1]), kEpsilon);
  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, input.max_accelerations));

  // Initial velocity is -1.0, maximum acceleration is 0.5, so it temporarily stops in 2 seconds
  EXPECT_TRUE(VerifyZeroVelocity(trajectory, 2.0, "stop state velocities"));
  Eigen::VectorXd stop_state_pos;
  stop_state_pos.resize(2);
  stop_state_pos << -1.0, 0.0;
  EXPECT_LT(CalcWayPointDistance(trajectory, stop_state_pos), kEpsilon);

  // Check an appropriate waypoint
  EXPECT_NEAR(trajectory.GetPosition(1.0)[0], -0.75, kEpsilon);
  EXPECT_NEAR(trajectory.GetPosition(1.0)[1], 0.0, kEpsilon);
  EXPECT_NEAR(trajectory.GetVelocity(1.0)[0], -0.5, kEpsilon);
  EXPECT_NEAR(trajectory.GetVelocity(1.0)[1], 0.0, kEpsilon);
}

// Optimization from an operational state, initial velocities of multiple joints
TEST(TrajectoryFilterViaStopStateTest, MultiJointsInitialVelocity) {
  auto input = TestInput();
  input.initial_velocities << -0.2, 0.5;

  auto trajectory = TrajectoryFilterViaStopState(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  ASSERT_TRUE(trajectory.IsValid());

  EXPECT_TRUE(VerifyEndVelocity(trajectory));
  EXPECT_LT(CalcWayPointDistance(trajectory, input.way_points[0]), 0.03 * 2);
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[1]), kEpsilon);
  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, input.max_accelerations));

  // Initial velocity of index1 is 0.5, maximum acceleration of 1.0 is effective
  EXPECT_TRUE(VerifyZeroVelocity(trajectory, 0.5, "stop state velocities"));
  Eigen::VectorXd stop_state_pos;
  stop_state_pos.resize(2);
  stop_state_pos << -0.05, 0.125;
  EXPECT_LT(CalcWayPointDistance(trajectory, stop_state_pos), kEpsilon);

  // Check an appropriate waypoint
  EXPECT_NEAR(trajectory.GetPosition(0.25)[0], -0.0375, kEpsilon);
  EXPECT_NEAR(trajectory.GetPosition(0.25)[1], 0.09375, kEpsilon);
  EXPECT_NEAR(trajectory.GetVelocity(0.25)[0], -0.1, kEpsilon);
  EXPECT_NEAR(trajectory.GetVelocity(0.25)[1], 0.25, kEpsilon);
}

// Change in acceleration ratio at stopping
TEST(TrajectoryFilterViaStopStateTest, AccelerationRateForStop) {
  auto input = TestInput();
  input.initial_velocities << input.max_velocities[0], 0.0;

  std::function<bool()> interrupt = []() -> bool{ return false; };
  auto trajectory = TrajectoryFilterViaStopState(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations, 2.0, interrupt);
  ASSERT_TRUE(trajectory.IsValid());

  EXPECT_TRUE(VerifyEndVelocity(trajectory));
  EXPECT_LT(CalcWayPointDistance(trajectory, input.way_points[0]), 0.03 * 2);
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[1]), kEpsilon);
  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));

  // Initial velocity is 1.0, maximum acceleration is 1.0, so it temporarily stops in 1 second
  EXPECT_TRUE(VerifyZeroVelocity(trajectory, 1.0, "stop state velocities"));
  Eigen::VectorXd stop_state_pos;
  stop_state_pos.resize(2);
  stop_state_pos << 0.5, 0.0;
  EXPECT_LT(CalcWayPointDistance(trajectory, stop_state_pos), kEpsilon);

  // Maximum acceleration differs before and after stopping
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, 0.0, 1.0, input.max_accelerations * 2.0));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, 1.0, trajectory.GetDuration(), input.max_accelerations));

  // Check an appropriate waypoint
  EXPECT_NEAR(trajectory.GetPosition(0.5)[0], 0.375, kEpsilon);
  EXPECT_NEAR(trajectory.GetPosition(0.5)[1], 0.0, kEpsilon);
  EXPECT_NEAR(trajectory.GetVelocity(0.5)[0], 0.5, kEpsilon);
  EXPECT_NEAR(trajectory.GetVelocity(0.5)[1], 0.0, kEpsilon);
}

// The stopping point and waypoint are the same
TEST(TrajectoryFilterViaStopStateTest, SameStopStateAndFirstWayPoint) {
  auto input = TestInput();
  input.initial_velocities << input.max_velocities[0], 0.0;

  Eigen::VectorXd stop_state_pos;
  stop_state_pos.resize(2);
  stop_state_pos << 1.0, 0.0;
  input.way_points.insert(input.way_points.begin(), stop_state_pos);

  auto trajectory = TrajectoryFilterViaStopState(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  ASSERT_TRUE(trajectory.IsValid());

  EXPECT_TRUE(VerifyEndVelocity(trajectory));
  EXPECT_LT(CalcWayPointDistance(trajectory, input.way_points[1]), 0.03 * 2);
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[2]), kEpsilon);
  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, input.max_accelerations));

  // Initial velocity is 1.0, maximum acceleration is 0.5, so it temporarily stops in 2 seconds
  EXPECT_TRUE(VerifyZeroVelocity(trajectory, 2.0, "stop state velocities"));
  EXPECT_LT(CalcWayPointDistance(trajectory, stop_state_pos), kEpsilon);

  // Check an appropriate waypoint
  EXPECT_NEAR(trajectory.GetPosition(1.0)[0], 0.75, kEpsilon);
  EXPECT_NEAR(trajectory.GetPosition(1.0)[1], 0.0, kEpsilon);
  EXPECT_NEAR(trajectory.GetVelocity(1.0)[0], 0.5, kEpsilon);
  EXPECT_NEAR(trajectory.GetVelocity(1.0)[1], 0.0, kEpsilon);
}

// There is one waypoint, and the stopping point and waypoint are the same
TEST(TrajectoryFilterViaStopStateTest, StopOnly) {
  auto input = TestInput();
  input.initial_velocities << input.max_velocities[0], 0.0;

  Eigen::VectorXd stop_state_pos;
  stop_state_pos.resize(2);
  stop_state_pos << 1.0, 0.0;
  input.way_points = {stop_state_pos};

  auto trajectory = TrajectoryFilterViaStopState(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  ASSERT_TRUE(trajectory.IsValid());

  // Initial velocity is 1.0, maximum acceleration is 0.5, so it temporarily stops in 2 seconds, and that is the playback duration of the trajectory
  EXPECT_NEAR(trajectory.GetDuration(), 2.0, kEpsilon);
  EXPECT_TRUE(VerifyZeroVelocity(trajectory, 2.0, "stop state velocities"));
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[0]), kEpsilon);

  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, input.max_accelerations));
}

}  // namespace tmc_timeopt

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
