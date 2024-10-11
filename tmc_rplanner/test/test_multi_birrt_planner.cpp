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
/// @brief    Multi_birrt_planner test

#include <ctime>
#include <stdlib.h>
#include <random>
#include <vector>
#include <gtest/gtest.h>

#include <tmc_rplanner/multi_birrt_planner.hpp>

using tmc_rplanner::Config;

namespace {
// State space used in the test
int32_t kDim = 2;
// Exploration
double kDelta = 0.02;
// Identity tolerance value of floating point
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

bool CheckFeasibility(const Config& config) {
  if (((config(0) < 0.5) && (config(0) > 0))
      && ((config(1) < 3.0) && (config(1) > 2.0))) return false;
  if (((config(0) < 4.0) && (config(0) > 3.5))
      && ((config(1) < 3.0) && (config(1) > 2.0))) return false;
  if (((config(0) < 3.0) && (config(0) > 2.0))
      && ((config(1) < 3.0) && (config(1) > 2.0))) return false;

    return true;
}


bool GenerateStart(Config& v) {
  v.resize(2);
  v(0) = Randd()*0.1;
  v(1) = Randd()*0.1;
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

// Planning success check
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

  // Check of PATH
  // The initial value is init
  ASSERT_DOUBLE_EQ(starts[0](0), path.front()(0));
  ASSERT_DOUBLE_EQ(starts[0](1), path.front()(1));

  // The terminal value is Goal
  ASSERT_DOUBLE_EQ(goals[0](0), path.back()(0));
  ASSERT_DOUBLE_EQ(goals[0](1), path.back()(1));

  // Distance is always below Kdelta and FEASIBLE
  Config old_config = init;
  for (Path::iterator config = ++(path.begin());
       config != path.end();
       ++config) {
    EXPECT_TRUE((*config - old_config).norm() <= kDelta + kDoubleEps);
    EXPECT_TRUE(CheckFeasibility(*config) );
    old_config = *config;
  }
}

// Test whether it supports multiple goals
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

    // Check of PATH
    // The initial value is init
    // The terminal value is Goal
    ASSERT_TRUE((path.front() == init1) || (path.front() == init2));
    ASSERT_TRUE((path.back() == goal1) || (path.back() == goal2));

    if (path.front() == init1) use_init1 = true;
    if (path.front() == init2) use_init2 = true;
    if (path.back() == goal1) use_goal1 = true;
    if (path.back() == goal2) use_goal2 = true;

    // Distance is always below Kdelta and FEASIBLE
    Config old_config = path.front();
    for (Path::iterator config = ++(path.begin());
         config != path.end();
         ++config) {
      EXPECT_TRUE((*config - old_config).norm() <= kDelta + kDoubleEps);
      EXPECT_TRUE(CheckFeasibility(*config));
      old_config = *config;
    }
  }
  // Check if you are using INIT and Goal of all patterns
  ASSERT_TRUE(use_init1 && use_init2 && use_goal1 && use_goal2);
}

// Test whether it supports multiple orbital generation
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
  // Check of PATH

  for (std::vector<Path>::iterator path = paths.begin();
       path != paths.end();
       ++path) {
    // The initial value is INIT1
    // The terminal value is Goal1 or Goal2 or Goal3
    ASSERT_TRUE(path->front() == init1);
    ASSERT_TRUE((path->back() == goal1) || (path->back() == goal2)
                || (path->back() == goal3));

    if (path->front() == init1) use_init1 = true;
    if (path->back() == goal1) use_goal1 = true;
    if (path->back() == goal2) use_goal2 = true;
    if (path->back() == goal3) use_goal3 = true;
    // Distance is always below Kdelta and FEASIBLE
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


// Can you plan with GENERATE START and Generate_goal?
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

    // Confirm that Start, Goal is different from the previous plan
    EXPECT_NE(old_init, path.front());
    EXPECT_NE(old_goal, path.back());

    old_init = path.front();
    old_goal = path.back();

    // std::cerr << "start = \n" << path.front() << std::endl;
    // std::cerr << "goal = \n" << path.back() << std::endl;

    // Distance is always below Kdelta and FEASIBLE
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


// Testing whether the value of max_connect is reflected
TEST_F(MultiBirrtPlannerTest, max_connect) {
  MultiBirrtPlannerParam param;
  param.delta = kDelta;
  param.max_itr = 10000;
  param.probability_start_generate = 0.0;
  param.probability_goal_generate = 0.0;
  param.max_connect = -1;

  planner_ = std::make_shared<MultiBirrtPlanner>(cspace_, param);

  // CONFIG [0] has no obstacle in (0.5, 2.0), so
  // You should be able to finish almost smoothly
  Config start(kDim);
  start << 1.0, 0.0;
  Config goal(kDim);
  goal << 1.0, 4.0;

  for (int32_t i = 0; i < 100; ++i) {
    Path path;
    ASSERT_EQ(kSuccess, planner_->PlanPath({start}, {goal}, path));
    // Start or goal should be connected from the Start or Goal to the opponent by extending the branch from the restricted Connect operation.
    // So, Config [0] enters 1.0 ± Kdelta
    for (const auto config : path) {
      EXPECT_LE(std::abs(config[0] - 1.0), kDelta);
    }
  }
}


// Ends with maximum number of repetitions
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

// End with the end condition function
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


// The initial value is not feasible
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

// The terminal value is not feast
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
