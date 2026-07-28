/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
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
/// @file     test_raund_robin_short_cutter.cpp
/// @brief    Test for raund_robin_short_cutter
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2011.12.01
/// @note     [1.0.0] 2011.12.01 Newly created

#include <stdlib.h>
#include <gtest/gtest.h>
#include <tmc_rplanner/birrt_planner.hpp>
#include <tmc_rplanner/raund_robin_short_cutter.hpp>

using tmc_rplanner::BiRrtPlanner;
using tmc_rplanner::CheckTransferabilityByDividing;
using tmc_rplanner::Collisions;
using tmc_rplanner::Config;
using tmc_rplanner::ConfigurationSpace;
using tmc_rplanner::DistanceFunc;
using tmc_rplanner::IPathShortCutter;
using tmc_rplanner::IPointToPointPlanner;
using tmc_rplanner::kSuccess;
using tmc_rplanner::Path;
using tmc_rplanner::RoundRobinShortCutter;

namespace {
// Dimension of the state space used in the test
int32_t kDim = 2;
// Search width
double kDelta = 0.2;
// Tolerance value for floating-point equality
double kDoubleEps = 1e-5;
// Acceptable trajectory length after shortcutting
double kLengthAcceptable = 16.0;

static double Randd() {
  static uint32_t seed = 2;
  return static_cast<double>(rand_r(&seed))/RAND_MAX;
}

Config RandomConfig() {
  Config v(2);
  v(0) = Randd()*4.0;
  v(1) = Randd()*4.0;
  return v;
}

bool CheckFeasibilityWithCollisions(const Config& config, std::vector<Collisions>& dst_collisions) {
  dst_collisions.clear();
  if (((config(0) < 3.5) && (config(0) > 0)) &&
      ((config(1) < 1.5) && (config(1) > 1.0))) {
    Collisions collision;
    collision.name_1 = "obstacle1";
    collision.name_2 = "robot";
    collision.depth = std::min(3.5 - config(0), config(0) - 0.0);
    collision.depth = std::min(collision.depth, 1.5 - config(1));
    collision.depth = std::min(collision.depth, config(1) - 1.0);
    dst_collisions.push_back(collision);
  }
  if (((config(0) < 4.0) && (config(0) > 0.5)) &&
      ((config(1) < 3.5) && (config(1) > 3.0))) {
    Collisions collision;
    collision.name_1 = "obstacle2";
    collision.name_2 = "robot";
    collision.depth = std::min(4.0 - config(0), config(0) - 0.5);
    collision.depth = std::min(collision.depth, 3.5 - config(1));
    collision.depth = std::min(collision.depth, config(1) - 3.0);
    dst_collisions.push_back(collision);
  }
  return true;
}

bool CheckFeasibility(const Config& config) {
  std::vector<Collisions> collisions;
  if (CheckFeasibilityWithCollisions(config, collisions)) {
    if (collisions.empty()) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool CheckTransferability(const Config& src_config,
                          const Config& dst_config,
                          const std::vector<Collisions>& src_collisions,
                          std::vector<Collisions>& dst_collisions) {
  return CheckTransferabilityByDividing(
      src_config, dst_config, src_collisions, CheckFeasibilityWithCollisions, DistanceFunc(), 0.01, dst_collisions);
}

bool IsTerminate() {
  return true;
}

double CalcLength(const Path& path) {
  double length = 0.0;
  for (size_t i = 0; i < path.size()-1; ++i) {
    length += (path[i] - path[i+1]).norm();
  }
  return length;
}

}  // anonymous namespace

///////////////////////////////////////////
/// raund_robin_short_cutter
///////////////////////////////////////////

// Check that the trajectory created by birrt becomes shorter
TEST(RaundRobinShortCutterTest, simple_shortcut) {
  ConfigurationSpace::Ptr cspace(new ConfigurationSpace(kDim));
  cspace->set_random_config(RandomConfig);
  cspace->set_check_feasibility(CheckFeasibility);
  IPointToPointPlanner::Ptr planner(new BiRrtPlanner(cspace, kDelta, 100000));

  Config init(kDim);
  init << 0.0, 0.0;
  Config goal(kDim);
  goal << 4.0, 4.0;

  Path rough_path;
  ASSERT_EQ(kSuccess, planner->PlanPath(init, goal, rough_path));

  IPathShortCutter::Ptr bi_short_cutter(new RoundRobinShortCutter(cspace, kDelta, true, 1));
  IPathShortCutter::Ptr mono_short_cutter(new RoundRobinShortCutter(cspace, kDelta, false, 1));

  Path bi_path;
  Path mono_path;

  bi_short_cutter->ShortCut(rough_path, bi_path);
  mono_short_cutter->ShortCut(rough_path, mono_path);

  double rough_length = CalcLength(rough_path);
  double bi_length = CalcLength(bi_path);
  double mono_length = CalcLength(mono_path);

  // Length will be rough > mono > bi
  EXPECT_TRUE(rough_length >= mono_length);
  EXPECT_TRUE(mono_length >= bi_length);
  // Length of bi is within kLengthAcceptable
  EXPECT_TRUE(bi_length <= kLengthAcceptable);

  // Check if the shortcut path is correct
  Config old_config = init;
  for (Path::iterator config = ++(mono_path.begin()); config != mono_path.end(); ++config) {
    EXPECT_TRUE((*config - old_config).norm() <= kDelta + kDoubleEps);
    EXPECT_TRUE(CheckFeasibility(*config));
    old_config = *config;
  }

  // Check if the shortcut path is correct
  old_config = init;
  for (Path::iterator config = ++(bi_path.begin()); config != bi_path.end(); ++config) {
    EXPECT_TRUE((*config - old_config).norm() <= kDelta + kDoubleEps);
    EXPECT_TRUE(CheckFeasibility(*config));
    old_config = *config;
  }
}

// Shortcutting of initial state collision trajectory
TEST(RaundRobinShortCutterTest, init_collision_shortcut) {
  ConfigurationSpace::Ptr cspace(new ConfigurationSpace(kDim));
  cspace->set_random_config(RandomConfig);
  cspace->set_check_feasibility(CheckFeasibility);
  cspace->set_check_feasibility_with_collisions(CheckFeasibilityWithCollisions);
  cspace->set_check_transferability(CheckTransferability);
  IPointToPointPlanner::Ptr planner(new BiRrtPlanner(cspace, kDelta, 100000));

  Config init(kDim);
  init << 0.5, 1.2;  // Inside the obstacle
  Config goal(kDim);
  goal << 4.0, 4.0;

  Path rough_path;
  ASSERT_EQ(kSuccess, planner->PlanPath(init, goal, rough_path));

  IPathShortCutter::Ptr bi_short_cutter(new RoundRobinShortCutter(cspace, kDelta, true, 1));
  IPathShortCutter::Ptr mono_short_cutter(new RoundRobinShortCutter(cspace, kDelta, false, 1));

  Path bi_path;
  Path mono_path;

  bi_short_cutter->ShortCut(rough_path, bi_path);
  mono_short_cutter->ShortCut(rough_path, mono_path);

  double rough_length = CalcLength(rough_path);
  double bi_length = CalcLength(bi_path);
  double mono_length = CalcLength(mono_path);

  // Length will be rough > mono > bi
  EXPECT_TRUE(rough_length >= mono_length);
  EXPECT_TRUE(mono_length >= bi_length);

  // Check if the shortcut path is correct
  Config old_config = init;
  std::vector<bool> feasibility_checks = {CheckFeasibility(init)};
  for (Path::iterator config = ++(mono_path.begin()); config != mono_path.end(); ++config) {
    EXPECT_TRUE((*config - old_config).norm() <= kDelta + kDoubleEps);
    feasibility_checks.push_back(CheckFeasibility(*config));
    old_config = *config;
  }
  EXPECT_FALSE(feasibility_checks.front());
  auto true_it = std::find(feasibility_checks.begin(), feasibility_checks.end(), true);
  EXPECT_TRUE(std::all_of(feasibility_checks.begin(), true_it, [](bool v) { return !v; }));
  EXPECT_TRUE(std::all_of(true_it, feasibility_checks.end(), [](bool v) { return v; }));

  // Check if the shortcut path is correct
  old_config = init;
  feasibility_checks = {CheckFeasibility(init)};
  for (Path::iterator config = ++(bi_path.begin()); config != bi_path.end(); ++config) {
    EXPECT_TRUE((*config - old_config).norm() <= kDelta + kDoubleEps);
    feasibility_checks.push_back(CheckFeasibility(*config));
    old_config = *config;
  }
  EXPECT_FALSE(feasibility_checks.front());
  true_it = std::find(feasibility_checks.begin(), feasibility_checks.end(), true);
  EXPECT_TRUE(std::all_of(feasibility_checks.begin(), true_it, [](bool v) { return !v; }));
  EXPECT_TRUE(std::all_of(true_it, feasibility_checks.end(), [](bool v) { return v; }));
}

// Empty exception throw test
TEST(RaundRobinShortCutterTest, short_cut_empty_exceptional) {
  ConfigurationSpace::Ptr cspace(new ConfigurationSpace(kDim));
  cspace->set_random_config(RandomConfig);
  cspace->set_check_feasibility(CheckFeasibility);
  IPathShortCutter::Ptr short_cutter(
      new RoundRobinShortCutter(cspace, kDelta, true, 1));
  Path empty_path;
  Path optim_path;
  EXPECT_THROW(short_cutter->ShortCut(empty_path, optim_path),
               std::invalid_argument);
}

// Negative skip exception test
TEST(RaundRobinShortCutterTest, short_cut_negative_skip_exceptional) {
  ConfigurationSpace::Ptr cspace(new ConfigurationSpace(kDim));
  cspace->set_random_config(RandomConfig);
  cspace->set_check_feasibility(CheckFeasibility);
  IPathShortCutter::Ptr short_cutter(new RoundRobinShortCutter(cspace, kDelta, true, -1));
  Path empty_path;
  Path optim_path;
  EXPECT_THROW(short_cutter->ShortCut(empty_path, optim_path),
               std::invalid_argument);
}

// Termination by termination condition function
TEST(RaundRobinShortCutterTest, terminate) {
  ConfigurationSpace::Ptr cspace(new ConfigurationSpace(kDim));
  cspace->set_random_config(RandomConfig);
  cspace->set_check_feasibility(CheckFeasibility);
  IPathShortCutter::Ptr short_cutter(
      new RoundRobinShortCutter(cspace, kDelta, true, 1, IsTerminate));

  IPointToPointPlanner::Ptr planner(new BiRrtPlanner(cspace, kDelta, 100000));

  Config init(kDim);
  init << 0.0, 0.0;
  Config goal(kDim);
  goal << 4.0, 4.0;

  Path path_in;
  Path path_out;
  ASSERT_EQ(kSuccess, planner->PlanPath(init, goal, path_in));
  short_cutter->ShortCut(path_in, path_out);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
