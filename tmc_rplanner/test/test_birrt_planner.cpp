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
/// @file     test_birrt_planner.cpp
/// @brief    Test of birrt_planner
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2011.12.01
/// @note     [1.0.0] 2011.12.01 Newly created

#include <stdlib.h>
#include <gtest/gtest.h>
#include <tmc_rplanner/birrt_planner.hpp>

using tmc_rplanner::BiRrtPlanner;
using tmc_rplanner::CheckTransferabilityByDividing;
using tmc_rplanner::Collisions;
using tmc_rplanner::Config;
using tmc_rplanner::ConfigurationSpace;
using tmc_rplanner::DistanceFunc;
using tmc_rplanner::IPointToPointPlanner;
using tmc_rplanner::kMaxItr;
using tmc_rplanner::kSuccess;
using tmc_rplanner::kTerminate;
using tmc_rplanner::Path;

namespace {
// Dimension of the state space used in the test
int32_t kDim = 2;
// Search width
double kDelta = 0.2;
// Tolerance value for floating-point equality
double kDoubleEps = 1e-5;


static double Randd() {
  static uint32_t seed = 1;
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

}  // anonymous namespace

///////////////////////////////////////////
/// birrt_test
///////////////////////////////////////////
class BiRrtPlannerTest : public ::testing::Test {
 protected:
  BiRrtPlannerTest() {
    const auto cspace = std::make_shared<ConfigurationSpace>(kDim);
    cspace->set_random_config(RandomConfig);
    cspace->set_check_feasibility(CheckFeasibility);
    cspace->set_check_feasibility_with_collisions(CheckFeasibilityWithCollisions);
    cspace->set_check_transferability(CheckTransferability);
    planner_ = std::make_shared<BiRrtPlanner>(cspace, kDelta, 10000);
  }
  IPointToPointPlanner::Ptr planner_;
};

// Planning success check
TEST_F(BiRrtPlannerTest, plan) {
  Config init(kDim);
  init << 0.0, 0.0;
  Config goal(kDim);
  goal << 4.0, 4.0;

  Path path;
  ASSERT_EQ(kSuccess, planner_->PlanPath(init, goal, path));

  // Path check
  // Initial value is init
  ASSERT_DOUBLE_EQ(init(0), path.front()(0));
  ASSERT_DOUBLE_EQ(init(1), path.front()(1));

  // Terminal value is goal
  ASSERT_DOUBLE_EQ(goal(0), path.back()(0));
  ASSERT_DOUBLE_EQ(goal(1), path.back()(1));

  // Distance is always less than or equal to kDelta and Feasible
  Config old_config = init;
  for (Path::iterator config = ++(path.begin()); config != path.end(); ++config) {
    EXPECT_LE((*config - old_config).norm(), kDelta + kDoubleEps);
    EXPECT_TRUE(CheckFeasibility(*config));
    old_config = *config;
  }
}

// Escape from initial state collision
TEST_F(BiRrtPlannerTest, init_collision) {
  Config init(kDim);
  init << 0.5, 1.2;  // Inside obstacle
  Config goal(kDim);
  goal << 4.0, 4.0;

  Path path;
  ASSERT_EQ(kSuccess, planner_->PlanPath(init, goal, path));

  // Path check
  // Initial value is init
  ASSERT_DOUBLE_EQ(init(0), path.front()(0));
  ASSERT_DOUBLE_EQ(init(1), path.front()(1));

  // Terminal value is goal
  ASSERT_DOUBLE_EQ(goal(0), path.back()(0));
  ASSERT_DOUBLE_EQ(goal(1), path.back()(1));

  // Escape from a state where distance is always less than or equal to kDelta and not Feasible to become Feasible
  Config old_config = init;
  std::vector<bool> feasibility_checks = {CheckFeasibility(init)};
  for (Path::iterator config = ++(path.begin()); config != path.end(); ++config) {
    EXPECT_LE((*config - old_config).norm(), kDelta + kDoubleEps);
    feasibility_checks.push_back(CheckFeasibility(*config));
    old_config = *config;
  }
  // The initial state is not Feasible, but it should become Feasible along the way
  EXPECT_FALSE(feasibility_checks.front());
  const auto true_it = std::find(feasibility_checks.begin(), feasibility_checks.end(), true);
  EXPECT_TRUE(std::all_of(feasibility_checks.begin(), true_it, [](bool v) { return !v; }));
  EXPECT_TRUE(std::all_of(true_it, feasibility_checks.end(), [](bool v) { return v; }));
}

// Failure if the target state is in collision
TEST_F(BiRrtPlannerTest, goal_collision) {
  Config init(kDim);
  init << 0.0, 0.0;
  Config goal(kDim);
  goal << 1.0, 1.2;  // Inside obstacle

  Path path;
  EXPECT_EQ(tmc_rplanner::kGoalConfigFail, planner_->PlanPath(init, goal, path));
}

// Termination at maximum number of iterations
TEST(BiRrtPlanner, max_itr) {
  ConfigurationSpace::Ptr cspace(new ConfigurationSpace(kDim));
  cspace->set_random_config(RandomConfig);
  cspace->set_check_feasibility(CheckFeasibility);
  IPointToPointPlanner::Ptr planner(new BiRrtPlanner(cspace, kDelta, 10));
  Path path;
  Config init(kDim);
  init << 0.0, 0.0;
  Config goal(kDim);
  goal << 4.0, 4.0;

  EXPECT_EQ(kMaxItr, planner->PlanPath(init, goal, path));
}

// Termination by end condition function
TEST(BiRrtPlanner, terminate) {
  ConfigurationSpace::Ptr cspace(new ConfigurationSpace(kDim));
  cspace->set_random_config(RandomConfig);
  cspace->set_check_feasibility(CheckFeasibility);
  IPointToPointPlanner::Ptr planner(new BiRrtPlanner(cspace, kDelta, 10, IsTerminate));
  Path path;
  Config init(kDim);
  init << 0.0, 0.0;
  Config goal(kDim);
  goal << 4.0, 4.0;

  EXPECT_EQ(kTerminate, planner->PlanPath(init, goal, path));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
