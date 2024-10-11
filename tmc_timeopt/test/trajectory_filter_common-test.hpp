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
#ifndef TMC_TIMEOPT_TEST_TRAJECTORY_FILTER_COMMON_TEST_HPP_
#define TMC_TIMEOPT_TEST_TRAJECTORY_FILTER_COMMON_TEST_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>

namespace tmc_timeopt {
constexpr double kEpsilon = 0.01;
constexpr double kTimeStep = 0.1;

Eigen::VectorXd ResizeInput(const Eigen::VectorXd& input, uint32_t new_size) {
  Eigen::VectorXd resized(new_size);
  auto copy_size = std::min(static_cast<uint32_t>(input.size()), new_size);
  resized.head(copy_size) = input.head(copy_size);
  return resized;
}

class InterruptionMock {
 public:
  explicit InterruptionMock(int32_t false_num): remainings_(false_num) {}

  std::function<bool()> get() {
    return std::function<bool()>([&]() { return mock(); });
  }

 private:
  int32_t remainings_;

  bool mock() {
    if (remainings_ < 0) {
      return false;
    } else if (remainings_ == 0) {
      return true;
    } else {
      --remainings_;
      return false;
    }
  }
};

double CalcWayPointDistance(const ITrajectoryFilter& trajectory,
                            Eigen::VectorXd& point) {
  double min_distance = DBL_MAX;
  for (double t = 0.0; t < trajectory.GetDuration(); t += kTimeStep) {
    auto distance = (trajectory.GetPosition(t) - point).norm();
    min_distance = std::min(distance, min_distance);
  }
  return min_distance;
}

double CalcLastPointDistnace(const ITrajectoryFilter& trajectory,
                             Eigen::VectorXd& point) {
  auto t = trajectory.GetDuration();
  auto last_point = trajectory.GetPosition(t);
  return (point - last_point).norm();
}

bool VerifyPointVelocity(const ITrajectoryFilter& trajectory,
                         double time_from_start,
                         const Eigen::VectorXd& target_velocities,
                         const std::string& name) {
  Eigen::VectorXd diff = trajectory.GetVelocity(time_from_start) - target_velocities;
  if (diff.norm() < kEpsilon) {
    return true;
  } else {
    std::cerr << "Invalid " << name << std::endl
              << "  Expected: " << target_velocities.transpose() << std::endl
              << "  Actual:   " << trajectory.GetVelocity(time_from_start).transpose() << std::endl;
    return false;
  }
}

bool VerifyZeroVelocity(const ITrajectoryFilter& trajectory, double time_from_start, const std::string& name) {
  const auto dof = trajectory.GetVelocity(0.0).size();
  return VerifyPointVelocity(trajectory, time_from_start, Eigen::VectorXd::Zero(dof), name);
}

bool VerifyEndVelocity(const ITrajectoryFilter& trajectory) {
  const auto dof = trajectory.GetVelocity(0.0).size();
  return VerifyZeroVelocity(trajectory, trajectory.GetDuration(), "last velocities");
}

bool VerifyBothEndVelocity(const ITrajectoryFilter& trajectory,
                           const Eigen::VectorXd& initial_velocities) {
  return (VerifyPointVelocity(trajectory, 0.0, initial_velocities, "initial velocities") &&
          VerifyPointVelocity(trajectory, trajectory.GetDuration(),
                              Eigen::VectorXd::Zero(initial_velocities.size()),
                              "last velocities"));
}

bool VerifyVelocityLimit(const ITrajectoryFilter& trajectory,
                         const Eigen::VectorXd& limits) {
  for (double t = 0.0; t < trajectory.GetDuration(); t += kTimeStep) {
    auto velocities = trajectory.GetVelocity(t);
    for (uint32_t i = 0; i < limits.size(); ++i) {
      // The test will not be stable without a margin because it is a comparison of floating points.
      if (fabs(velocities[i]) > limits[i] + kEpsilon) {
        std::cerr << t << ": " << velocities.transpose() << std::endl;
        return false;
      }
    }
  }
  return true;
}

bool VerifyAccelarationLimitImpl(const ITrajectoryFilter& trajectory,
                                 double start_stamp,
                                 double end_stamp,
                                 const Eigen::VectorXd& limits) {
  auto start_velocities = trajectory.GetVelocity(start_stamp);
  auto end_velocities = trajectory.GetVelocity(end_stamp);
  auto accelarations = (end_velocities - start_velocities) / (end_stamp - start_stamp);
  for (uint32_t i = 0; i < limits.size(); ++i) {
    if (fabs(accelarations[i]) > limits[i] + kEpsilon) {
      std::cerr << start_stamp << ": " << accelarations.transpose() << std::endl;
      return false;
    }
  }
  return true;
}

bool VerifyAccelarationLimit(const ITrajectoryFilter& trajectory,
                             double start_stamp,
                             double end_stamp,
                             const Eigen::VectorXd& limits) {
  for (double t = start_stamp; t < end_stamp; t += kTimeStep) {
    if (!VerifyAccelarationLimitImpl(trajectory, t, t + kTimeStep, limits)) {
      return false;
    }
  }
  return VerifyAccelarationLimitImpl(trajectory, end_stamp - kTimeStep, end_stamp, limits);
}

bool VerifyAccelarationLimit(const ITrajectoryFilter& trajectory,
                             const Eigen::VectorXd& limits) {
  return VerifyAccelarationLimit(trajectory, 0.0, trajectory.GetDuration(), limits);
}

struct TestInput {
  Eigen::VectorXd initial_positions;
  Eigen::VectorXd initial_velocities;
  std::vector<Eigen::VectorXd> way_points;
  Eigen::VectorXd max_velocities;
  Eigen::VectorXd max_accelerations;

  TestInput() {
    initial_positions.resize(2);
    initial_positions << 0.0, 0.0;
    initial_velocities.resize(2);
    initial_velocities << 0.0, 0.0;
    way_points.resize(2);
    way_points[0].resize(2);
    way_points[0] << 3.0, 2.0;
    way_points[1].resize(2);
    way_points[1] << 4.0, 1.0;
    max_velocities.resize(2);
    max_velocities << 1.0, 0.5;
    max_accelerations.resize(2);
    max_accelerations << 0.5, 1.0;
  }
};

template <typename FilterType>
class TrajectoryFilterCommonTest : public ::testing::Test {};

TYPED_TEST_SUITE_P(TrajectoryFilterCommonTest);

// Testing to fail when the speed limit contains non -correct values
TYPED_TEST_P(TrajectoryFilterCommonTest, MaxVelocityNotPositive) {
  std::vector<std::array<double, 2>> test_cases = {
      {1.0, 0.0}, {0.0, 0.5}, {0.0, 0.0}, {1.0, -0.5}, {-1.0, 0.5}, {-1.0, -0.5}};
  for (const auto max_velocities : test_cases) {
    auto input = TestInput();
    input.max_velocities << max_velocities[0], max_velocities[1];
    auto trajectory = TypeParam(
        input.initial_positions, input.initial_velocities, input.way_points,
        input.max_velocities, input.max_accelerations);
    EXPECT_FALSE(trajectory.IsValid());
  }
}

// Testing to fail if the acceleration limit contains non -correct values
TYPED_TEST_P(TrajectoryFilterCommonTest, MaxAccelarationNotPositive) {
  std::vector<std::array<double, 2>> test_cases = {
      {0.5, 0.0}, {0.0, 1.0}, {0.0, 0.0}, {0.5, -1.0}, {-0.5, 1.0}, {-0.5, -1.0}};
  for (const auto max_accelarations : test_cases) {
    auto input = TestInput();
    input.max_accelerations << max_accelarations[0], max_accelarations[1];
    auto trajectory = TypeParam(
        input.initial_positions, input.initial_velocities, input.way_points,
        input.max_velocities, input.max_accelerations);
    EXPECT_FALSE(trajectory.IsValid());
  }
}

// Test of freedom disagreement
TYPED_TEST_P(TrajectoryFilterCommonTest, DofMismatch) {
  std::vector<std::array<uint32_t, 6>> test_cases = {
      {3, 2, 2, 2, 2, 2}, {2, 3, 2, 2, 2, 2}, {2, 2, 3, 2, 2, 2},
      {2, 2, 2, 3, 2, 2}, {2, 2, 2, 2, 3, 2}, {2, 2, 2, 2, 2, 3},
      {1, 2, 2, 2, 2, 2}, {2, 1, 2, 2, 2, 2}, {2, 2, 1, 2, 2, 2},
      {2, 2, 2, 1, 2, 2}, {2, 2, 2, 2, 1, 2}, {2, 2, 2, 2, 2, 1}};
  for (const auto input_sizes : test_cases) {
    auto input = TestInput();
    input.initial_positions = ResizeInput(input.initial_positions, input_sizes[0]);
    input.initial_velocities = ResizeInput(input.initial_velocities, input_sizes[1]);
    input.way_points[0] = ResizeInput(input.way_points[0], input_sizes[2]);
    input.way_points[1] = ResizeInput(input.way_points[1], input_sizes[3]);
    input.max_velocities = ResizeInput(input.max_velocities, input_sizes[4]);
    input.max_accelerations = ResizeInput(input.max_accelerations, input_sizes[5]);
    auto trajectory = TypeParam(
        input.initial_positions, input.initial_velocities, input.way_points,
        input.max_velocities, input.max_accelerations);
    EXPECT_FALSE(trajectory.IsValid());
  }
}

// Route is an empty test
TYPED_TEST_P(TrajectoryFilterCommonTest, EmptyWayPoints) {
  auto input = TestInput();
  input.way_points.clear();
  auto trajectory = TypeParam(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  EXPECT_FALSE(trajectory.IsValid());
}

// 1 Optimization of trajectory of freedom
TYPED_TEST_P(TrajectoryFilterCommonTest, OneDofTrajectory) {
  auto input = TestInput();
  input.initial_positions.resize(1);
  input.initial_velocities.resize(1);
  input.way_points[0].resize(1);
  input.way_points[1].resize(1);
  input.max_velocities.resize(1);
  input.max_accelerations.resize(1);
  auto trajectory = TypeParam(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  EXPECT_FALSE(trajectory.IsValid());
}

// Interrupt test
TYPED_TEST_P(TrajectoryFilterCommonTest, Interrupt) {
  auto input = TestInput();
  InterruptionMock return_false_mock(-1);
  std::function<bool()> mock_func = return_false_mock.get();
  auto trajectory = TypeParam(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations, mock_func);
  EXPECT_TRUE(trajectory.IsValid());

  InterruptionMock delay_mock(1);
  mock_func = delay_mock.get();
  trajectory = TypeParam(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations, mock_func);
  EXPECT_FALSE(trajectory.IsValid());
}

// Optimization of orbit including the same points for continuous vouchers
TYPED_TEST_P(TrajectoryFilterCommonTest, SeriallySameWayPoint) {
  auto input = TestInput();
  input.way_points.insert(input.way_points.begin(), input.way_points.front());
  input.initial_velocities << 1.0, 0.5;
  auto trajectory = TypeParam(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  EXPECT_TRUE(trajectory.IsValid());

  EXPECT_TRUE(VerifyBothEndVelocity(trajectory, input.initial_velocities));
  // Kmaxdeviation = 0.03, so it should be in that degree.
  EXPECT_LT(CalcWayPointDistance(trajectory, input.way_points[1]), 0.03 * 2);
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[2]), kEpsilon);
  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, input.max_accelerations));
}

// Optimization of the same point of initial posture and voting points
TYPED_TEST_P(TrajectoryFilterCommonTest, SameInitialPositionAndFirstWayPoint) {
  auto input = TestInput();
  input.way_points.insert(input.way_points.begin(), input.initial_positions);
  input.initial_velocities << 1.0, 0.5;
  auto trajectory = TypeParam(
      input.initial_positions, input.initial_velocities, input.way_points,
      input.max_velocities, input.max_accelerations);
  EXPECT_TRUE(trajectory.IsValid());

  EXPECT_TRUE(VerifyBothEndVelocity(trajectory, input.initial_velocities));
  EXPECT_LT(CalcWayPointDistance(trajectory, input.way_points[1]), 0.03 * 2);
  EXPECT_LT(CalcLastPointDistnace(trajectory, input.way_points[2]), kEpsilon);
  EXPECT_TRUE(VerifyVelocityLimit(trajectory, input.max_velocities));
  EXPECT_TRUE(VerifyAccelarationLimit(trajectory, input.max_accelerations));
}

REGISTER_TYPED_TEST_SUITE_P(TrajectoryFilterCommonTest,
                            MaxVelocityNotPositive,
                            MaxAccelarationNotPositive,
                            DofMismatch,
                            EmptyWayPoints,
                            OneDofTrajectory,
                            Interrupt,
                            SeriallySameWayPoint,
                            SameInitialPositionAndFirstWayPoint);

}  // namespace tmc_timeopt

#endif  // TMC_TIMEOPT_TEST_TRAJECTORY_FILTER_TEST_UTILS_HPP_
