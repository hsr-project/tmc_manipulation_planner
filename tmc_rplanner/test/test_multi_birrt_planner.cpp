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
/// @file     test_multi_multi_birrt_planner.cpp
/// @brief    Test for multi_birrt_planner
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2012.04.02

#include <ctime>
#include <stdlib.h>
#include <random>
#include <vector>
#include <gtest/gtest.h>

#include <tmc_rplanner/multi_birrt_planner.hpp>

using tmc_rplanner::CheckTransferabilityByDividing;
using tmc_rplanner::Collisions;
using tmc_rplanner::Config;
using tmc_rplanner::DistanceFunc;

namespace {
// Dimension of the state space used in the test
int32_t kDim = 2;
// Search width
double kDelta = 0.02;
// Tolerance for floating-point equality
double kDoubleEps = 1e-5;


double Randd() {
  static std::mt19937 eng(static_cast<uint32_t>(std::time(0)));
  std::uniform_real_distribution<> randf(0.0, 1.0);

  return randf(eng);
}

Config RandomConfig() {
  Config v(2);
  v(0) = Randd()*4.0;
  v(1) = Randd()*4.0;
  return v;
}

bool CheckFeasibilityWithCollisions(const Config& config, std::vector<Collisions>& dst_collisions) {
  dst_collisions.clear();
  if (((config(0) < 0.5) && (config(0) > 0))
      && ((config(1) < 3.0) && (config(1) > 2.0))) {
    Collisions collision;
    collision.name_1 = "obstacle1";
    collision.name_2 = "robot";
    collision.depth = std::min(0.5 - config(0), config(0) - 0.0);
    collision.depth = std::min(collision.depth, 3.0 - config(1));
    collision.depth = std::min(collision.depth, config(1) - 2.0);
    dst_collisions.push_back(collision);
  }
  if (((config(0) < 4.0) && (config(0) > 3.5))
      && ((config(1) < 3.0) && (config(1) > 2.0))) {
    Collisions collision;
    collision.name_1 = "obstacle2";
    collision.name_2 = "robot";
    collision.depth = std::min(4.0 - config(0), config(0) - 3.5);
    collision.depth = std::min(collision.depth, 3.0 - config(1));
    collision.depth = std::min(collision.depth, config(1) - 2.0);
    dst_collisions.push_back(collision);
  }
  if (((config(0) < 3.0) && (config(0) > 2.0))
      && ((config(1) < 3.0) && (config(1) > 2.0))) {
    Collisions collision;
    collision.name_1 = "obstacle3";
    collision.name_2 = "robot";
    collision.depth = std::min(3.0 - config(0), config(0) - 2.0);
    collision.depth = std::min(collision.depth, 3.0 - config(1));
    collision.depth = std::min(collision.depth, config(1) - 2.0);
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

bool GenerateStart(Config& v) {
  v.resize(2);
  v(0) = Randd()*0.1;
  v(1) = Randd()*0.1;
  return true;
}

bool GenerateInCollisionConfig(Config& v) {
  v.resize(2);
  v(0) = 0.25;
  v(1) = 2.5 + Randd() * 0.1;
  return true;
}

bool GenerateGoal(Config& v) {
  v.resize(2);
  v(0) = 3.9+Randd()*0.1;
  v(1) = 3.9+Randd()*0.1;
  return true;
}


bool IsTerminate() {
  return true;
}
}  // anonymous namespace

namespace tmc_rplanner {
///////////////////////////////////////////
/// multi birrt_test
///////////////////////////////////////////
class MultiBirrtPlannerTest : public ::testing::Test {
 protected:
  MultiBirrtPlannerTest() {
    cspace_ = std::make_shared<ConfigurationSpace>(kDim);
    cspace_->set_random_config(RandomConfig);
    cspace_->set_check_feasibility(CheckFeasibility);
  }
  ConfigurationSpace::Ptr cspace_;
  IMultiPlanner::Ptr planner_;
};

// Check for planning success
TEST_F(MultiBirrtPlannerTest, plan) {
  MultiBirrtPlannerParam param;
  param.delta = kDelta;
  param.max_itr = 10000;
  param.probability_start_generate = 0.0;
  param.probability_goal_generate = 0.0;

  planner_ = std::make_shared<MultiBirrtPlanner>(cspace_, param);

  Config init(kDim);
  init << 0.0, 0.0;
  Config goal(kDim);
  goal << 4.0, 4.0;

  std::vector<Config> starts;
  std::vector<Config> goals;
  starts.push_back(init);
  goals.push_back(goal);

  Path path;
  ASSERT_EQ(kSuccess, planner_->PlanPath(starts, goals, path));

  // Check the path
  // Initial value is init
  ASSERT_DOUBLE_EQ(starts[0](0), path.front()(0));
  ASSERT_DOUBLE_EQ(starts[0](1), path.front()(1));

  // Terminal value is goal
  ASSERT_DOUBLE_EQ(goals[0](0), path.back()(0));
  ASSERT_DOUBLE_EQ(goals[0](1), path.back()(1));

  // Distance is always within kDelta and Feasible
  Config old_config = init;
  for (Path::iterator config = ++(path.begin());
       config != path.end();
       ++config) {
    EXPECT_TRUE((*config - old_config).norm() <= kDelta + kDoubleEps);
    EXPECT_TRUE(CheckFeasibility(*config) );
    old_config = *config;
  }
}

// Escape from initial state collision
TEST_F(MultiBirrtPlannerTest, init_collision) {
  MultiBirrtPlannerParam param;
  param.delta = kDelta;
  param.max_itr = 10000;
  param.probability_start_generate = 0.0;
  param.probability_goal_generate = 0.0;

  cspace_->set_generate_start_config(GenerateInCollisionConfig);
  cspace_->set_generate_goal_config(GenerateGoal);
  cspace_->set_check_feasibility_with_collisions(CheckFeasibilityWithCollisions);
  cspace_->set_check_transferability(CheckTransferability);
  planner_ = std::make_shared<MultiBirrtPlanner>(cspace_, param);

  Path path;
  ASSERT_EQ(kSuccess, planner_->PlanPath({}, {}, path));

  // Escape from a state where distance is always within kDelta but not Feasible, and become Feasible
  Config old_config = path.front();
  std::vector<bool> feasibility_checks = {CheckFeasibility(old_config)};
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

// Cannot escape from goal state collision
TEST_F(MultiBirrtPlannerTest, goal_collision) {
  MultiBirrtPlannerParam param;
  param.delta = kDelta;
  param.max_itr = 10000;
  param.probability_start_generate = 0.0;
  param.probability_goal_generate = 0.0;

  cspace_->set_generate_start_config(GenerateStart);
  cspace_->set_generate_goal_config(GenerateInCollisionConfig);
  cspace_->set_check_feasibility_with_collisions(CheckFeasibilityWithCollisions);
  cspace_->set_check_transferability(CheckTransferability);
  planner_ = std::make_shared<MultiBirrtPlanner>(cspace_, param);

  Path path;
  ASSERT_EQ(kGoalConfigFail, planner_->PlanPath({}, {}, path));
}

// Test if it supports multiple goals
TEST_F(MultiBirrtPlannerTest, multi_goal_test) {
  MultiBirrtPlannerParam param;
  param.delta = kDelta;
  param.max_itr = 10000;
  param.probability_start_generate = 0.0;
  param.probability_goal_generate = 0.0;

  planner_ = std::make_shared<MultiBirrtPlanner>(cspace_, param);

  Config init1(kDim);
  init1 << 0.0, 0.0;
  Config init2(kDim);
  init2 << 4.0, 0.0;

  Config goal1(kDim);
  goal1 << 4.0, 4.0;

  Config goal2(kDim);
  goal2 << 0.0, 4.0;

  std::vector<Config> starts;
  std::vector<Config> goals;
  starts.push_back(init1);
  goals.push_back(goal1);

  starts.push_back(init2);
  goals.push_back(goal2);

  bool use_init1 = false;
  bool use_init2 = false;
  bool use_goal1 = false;
  bool use_goal2 = false;

  for (int32_t i = 0; i < 10; ++i) {
    Path path;

    ASSERT_EQ(kSuccess, planner_->PlanPath(starts, goals, path));

    // Check the path
    // Initial value is init
    // Terminal value is goal
    ASSERT_TRUE((path.front() == init1) || (path.front() == init2));
    ASSERT_TRUE((path.back() == goal1) || (path.back() == goal2));

    if (path.front() == init1) use_init1 = true;
    if (path.front() == init2) use_init2 = true;
    if (path.back() == goal1) use_goal1 = true;
    if (path.back() == goal2) use_goal2 = true;

    // Distance is always within kDelta and Feasible
    Config old_config = path.front();
    for (Path::iterator config = ++(path.begin());
         config != path.end();
         ++config) {
      EXPECT_TRUE((*config - old_config).norm() <= kDelta + kDoubleEps);
      EXPECT_TRUE(CheckFeasibility(*config));
      old_config = *config;
    }
  }
  // Check if all patterns of init and goal are used
  ASSERT_TRUE(use_init1 && use_init2 && use_goal1 && use_goal2);
}

// Test if it supports multiple trajectory generation
TEST_F(MultiBirrtPlannerTest, multi_path_test) {
  MultiBirrtPlannerParam param;
  param.delta = kDelta;
  param.max_itr = 10000;
  param.probability_start_generate = 0.0;
  param.probability_goal_generate = 0.0;

  planner_ = std::make_shared<MultiBirrtPlanner>(cspace_, param);

  Config init1(kDim);
  init1 << 0.0, 0.0;

  Config goal1(kDim);
  goal1 << 4.0, 4.0;

  Config goal2(kDim);
  goal2 << 0.0, 4.0;

  Config goal3(kDim);
  goal3 << 2.0, 4.0;


  std::vector<Config> starts;
  std::vector<Config> goals;
  starts.push_back(init1);

  goals.push_back(goal1);
  goals.push_back(goal2);
  goals.push_back(goal3);

  bool use_init1 = false;
  bool use_goal1 = false;
  bool use_goal2 = false;
  bool use_goal3 = false;

  std::vector<Path> paths;
  ASSERT_EQ(kSuccess, planner_->PlanPaths(starts, goals, 3, paths));

  EXPECT_EQ(3, paths.size());
  // Check the path

  for (std::vector<Path>::iterator path = paths.begin();
       path != paths.end();
       ++path) {
    // Initial value is init1
    // Terminal value is goal1 or goal2 or goal3
    ASSERT_TRUE(path->front() == init1);
    ASSERT_TRUE((path->back() == goal1) || (path->back() == goal2)
                || (path->back() == goal3));

    if (path->front() == init1) use_init1 = true;
    if (path->back() == goal1) use_goal1 = true;
    if (path->back() == goal2) use_goal2 = true;
    if (path->back() == goal3) use_goal3 = true;
    // Distance is always within kDelta and Feasible
    Config old_config = path->front();
    for (Path::iterator config = ++(path->begin());
         config != path->end();
         ++config) {
      EXPECT_TRUE((*config - old_config).norm() <= kDelta + kDoubleEps);
      EXPECT_TRUE(CheckFeasibility(*config));
      old_config = *config;
    }
  }
  EXPECT_TRUE(use_init1);
  EXPECT_TRUE(use_goal1);
  EXPECT_TRUE(use_goal2);
  EXPECT_TRUE(use_goal3);
}


// Check if planning is possible with generate_start and generate_goal
TEST_F(MultiBirrtPlannerTest, generate_test) {
  MultiBirrtPlannerParam param;
  param.delta = kDelta;
  param.max_itr = 10000;
  param.probability_start_generate = 0.1;
  param.probability_goal_generate = 0.1;
  cspace_->set_generate_start_config(GenerateStart);
  cspace_->set_generate_goal_config(GenerateGoal);

  planner_ = std::make_shared<MultiBirrtPlanner>(cspace_, param);

  std::vector<Config> starts;
  std::vector<Config> goals;

  Config old_init(2);
  Config old_goal(2);


  for (int32_t i = 0; i < 10; ++i) {
    Path path;

    ASSERT_EQ(kSuccess, planner_->PlanPath(starts, goals, path));

    // Confirm that the path has a certain length
    EXPECT_LT(10, path.size());

    // Confirm that the start and goal are different from the previous plan
    EXPECT_NE(old_init, path.front());
    EXPECT_NE(old_goal, path.back());

    old_init = path.front();
    old_goal = path.back();

    // std::cerr << "start = \n" << path.front() << std::endl;
    // std::cerr << "goal = \n" << path.back() << std::endl;

    // Distance is always within kDelta and Feasible
    Config old_config = path.front();
    for (Path::iterator config = ++(path.begin());
         config != path.end();
         ++config) {
      EXPECT_TRUE((*config - old_config).norm() <= kDelta + kDoubleEps);
      EXPECT_TRUE(CheckFeasibility(*config));
      old_config = *config;
    }
  }
}


// Test if the value of max_connect is reflected
TEST_F(MultiBirrtPlannerTest, max_connect) {
  MultiBirrtPlannerParam param;
  param.delta = kDelta;
  param.max_itr = 10000;
  param.probability_start_generate = 0.0;
  param.probability_goal_generate = 0.0;
  param.max_connect = -1;

  planner_ = std::make_shared<MultiBirrtPlanner>(cspace_, param);

  // Since config[0] is set to have no obstacles at (0.5, 2.0),
  // It should be able to reach the goal almost straight
  Config start(kDim);
  start << 1.0, 0.0;
  Config goal(kDim);
  goal << 1.0, 4.0;

  for (int32_t i = 0; i < 100; ++i) {
    Path path;
    ASSERT_EQ(kSuccess, planner_->PlanPath({start}, {goal}, path));
    // Without restrictions, it should connect from start or goal to the other side with one branch extension
    // Therefore, config[0] should fall within 1.0±kDelta
    for (const auto config : path) {
      EXPECT_LE(std::abs(config[0] - 1.0), kDelta);
    }
  }
}


// Termination due to maximum iteration count
TEST_F(MultiBirrtPlannerTest, max_itr) {
  MultiBirrtPlannerParam param;
  param.delta = kDelta;
  param.max_itr = 5;
  param.probability_start_generate = 0.0;
  param.probability_goal_generate = 0.0;

  Config init(kDim);
  init << 0.0, 0.0;
  Config goal(kDim);
  goal << 4.0, 4.0;

  std::vector<Config> starts;
  std::vector<Config> goals;
  starts.push_back(init);
  goals.push_back(goal);

  planner_ = std::make_shared<MultiBirrtPlanner>(cspace_, param);
  Path path;
  EXPECT_EQ(kMaxItr, planner_->PlanPath(starts, goals, path));
}

// Termination by termination condition function
TEST_F(MultiBirrtPlannerTest, terminate) {
  MultiBirrtPlannerParam param;
  param.delta = kDelta;
  param.max_itr = 10000;
  param.probability_start_generate = 0.0;
  param.probability_goal_generate = 0.0;

  Config init(kDim);
  init << 0.0, 0.0;
  Config goal(kDim);
  goal << 4.0, 4.0;

  std::vector<Config> starts;
  std::vector<Config> goals;
  starts.push_back(init);
  goals.push_back(init);

  param.is_terminate = IsTerminate;
  planner_ = std::make_shared<MultiBirrtPlanner>(cspace_, param);
  Path path;
  EXPECT_EQ(kTerminate, planner_->PlanPath(starts, goals, path));
}


// Initial value is not feasible
TEST_F(MultiBirrtPlannerTest, init_config_fail) {
  MultiBirrtPlannerParam param;
  param.delta = kDelta;
  param.max_itr = 10000;
  param.probability_start_generate = 0.0;
  param.probability_goal_generate = 0.0;
  planner_ = std::make_shared<MultiBirrtPlanner>(cspace_, param);

  Config init(kDim);
  init << 0.01, 2.5;
  Config goal(kDim);
  goal << 4.0, 4.0;

  std::vector<Config> starts;
  std::vector<Config> goals;
  starts.push_back(init);
  goals.push_back(goal);

  Path path;
  ASSERT_EQ(kInitConfigFail, planner_->PlanPath(starts, goals, path));
}

// Terminal value is not feasible
TEST_F(MultiBirrtPlannerTest, goal_config_fail) {
  MultiBirrtPlannerParam param;
  param.delta = kDelta;
  param.max_itr = 10000;
  param.probability_start_generate = 0.0;
  param.probability_goal_generate = 0.0;

  planner_ = std::make_shared<MultiBirrtPlanner>(cspace_, param);

  Config init(kDim);
  init << 0.0, 0.0;
  Config goal(kDim);
  goal << 3.99, 2.5;

  std::vector<Config> starts;
  std::vector<Config> goals;
  starts.push_back(init);
  goals.push_back(goal);

  Path path;
  ASSERT_EQ(kGoalConfigFail, planner_->PlanPath(starts, goals, path));
}
}  // namespace tmc_rplanner

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
