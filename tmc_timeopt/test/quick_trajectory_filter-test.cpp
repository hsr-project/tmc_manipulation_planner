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
/// @brief Test of QuickTrajectoryFilter

#include <gtest/gtest.h>

#include <tmc_timeopt/quick_trajectory_filter.hpp>

#include "trajectory_filter_common-test.hpp"

namespace tmc_timeopt {

INSTANTIATE_TYPED_TEST_SUITE_P(QuickTrajectoryFilterTest, TrajectoryFilterCommonTest, QuickTrajectoryFilter);

// Test to check if optimization satisfies constraints even when initial and maximum velocities are changed
class OptimizationTest
    : public ::testing::TestWithParam<std::tuple<double, double, double> > {};

INSTANTIATE_TEST_SUITE_P(
    OptimizationTest, OptimizationTest,
    ::testing::Combine(::testing::Values(1.0, 0.0, -1.0),
                       ::testing::Values(1.0, 0.0, -1.0),
                       ::testing::Values(0.1, 1.0, 2.0)));

TEST_P(OptimizationTest, TestCase) {
  auto input = TestInput();
  input.max_velocities *= std::get<2>(GetParam());
  input.initial_velocities << input.max_velocities[0] * std::get<0>(GetParam()),
                              input.max_velocities[1] * std::get<1>(GetParam());

  auto trajectory = QuickTrajectoryFilter(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  ASSERT_TRUE(trajectory.IsValid());

  EXPECT_TRUE(VerifyBothEndVelocity(trajectory, input.initial_velocities));
  // Since kMaxDeviation = 0.03, it should be within that range
  EXPECT_LT(CalcWayPointDistance(trajectory, input.way_points[0]), 0.03 * 2);
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[1]), kEpsilon);
  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, input.max_accelerations));
}

// Test where the absolute value of the dot product of the unit vector in the direction of travel exceeds 1.0
TEST(QuickTrajectoryFilterTest, InvalidArcCosInput) {
  auto input = TestInput();
  // The issue where the absolute value of the dot product of the unit vector exceeds 1.0 due to calculation fluctuations does not occur when theta = 1.0
  // It occurs when starting from theta = 0.0 and incrementing by +=0.1 ten times
  double theta = 0.0;
  for (uint32_t i = 0; i < 10; ++i) {
    theta += 0.1;
  }
  input.way_points[0] << cos(theta), sin(theta);
  input.way_points[1] << -cos(theta), -sin(theta);
  auto trajectory = QuickTrajectoryFilter(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  EXPECT_TRUE(trajectory.IsValid());
}

// Optimization of a trajectory including waypoints where the direction of travel is completely opposite, even degrees of freedom
TEST(QuickTrajectoryFilterTest, ReverseTrajectory2Dof) {
  auto input = TestInput();
  input.way_points[1] = 0.5 * input.way_points[0];
  auto trajectory = QuickTrajectoryFilter(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  EXPECT_TRUE(trajectory.IsValid());

  EXPECT_TRUE(VerifyBothEndVelocity(trajectory, input.initial_velocities));
  EXPECT_LT(CalcWayPointDistance(trajectory, input.way_points[0]), 0.03 * 2);
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[1]), kEpsilon);
  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, input.max_accelerations));
}

// Optimization of a trajectory including waypoints where the direction of travel is completely opposite, odd degrees of freedom
TEST(QuickTrajectoryFilterTest, ReverseTrajectory3Dof) {
  auto input = TestInput();
  input.initial_positions = Eigen::VectorXd::Zero(3);
  input.initial_velocities = Eigen::VectorXd::Zero(3);
  input.way_points[0].resize(3);
  input.way_points[0] << 3.0, 2.0, 2.0;
  input.way_points[1] = 0.5 * input.way_points[0];
  input.max_velocities.resize(3);
  input.max_velocities << 1.0, 0.5, 0.5;
  input.max_accelerations.resize(3);
  input.max_accelerations << 0.5, 1.0, 1.0;
  auto trajectory = QuickTrajectoryFilter(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  EXPECT_TRUE(trajectory.IsValid());

  EXPECT_TRUE(VerifyBothEndVelocity(trajectory, input.initial_velocities));
  EXPECT_LT(CalcWayPointDistance(trajectory, input.way_points[0]), 0.03 * 2);
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[1]), kEpsilon);
  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, input.max_accelerations));
}

// Although it is a trajectory with multiple degrees of freedom, only the optimization of a trajectory with one degree of freedom is functioning
TEST(QuickTrajectoryFilterTest, Hoge) {
  auto input = TestInput();
  input.way_points[0][0] = 0.5;
  input.way_points[0][1] = 0.0;
  input.way_points[1][0] = 1.0;
  input.way_points[1][1] = 0.0;
  input.max_velocities[0] = 0.2;
  auto trajectory = QuickTrajectoryFilter(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  EXPECT_TRUE(trajectory.IsValid());

  EXPECT_TRUE(VerifyBothEndVelocity(trajectory, input.initial_velocities));
  EXPECT_LT(CalcWayPointDistance(trajectory, input.way_points[0]), 0.03 * 2);
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[1]), kEpsilon);
  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, input.max_accelerations));
}

}  // namespace tmc_timeopt

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
