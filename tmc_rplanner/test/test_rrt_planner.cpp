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
/// @brief    RRT_planner test

#include <stdlib.h>
#include <gtest/gtest.h>
#include <tmc_rplanner/rrt_planner.hpp>

using tmc_rplanner::Config;
using tmc_rplanner::ConfigurationSpace;
using tmc_rplanner::IPointToConditionPlanner;
using tmc_rplanner::kMaxItr;
using tmc_rplanner::kSuccess;
using tmc_rplanner::kTerminate;
using tmc_rplanner::Path;
using tmc_rplanner::RrtPlanner;

namespace {
// State space used in the test
int32_t kDim = 2;
// Exploration
double kDelta = 0.2;
// Identity tolerance value of floating point
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


bool CheckFeasibility(const Config& config) {
  // [0.5 3.5], [0.5 1.0].
  if (((config(0) > 0.5) && (config(0) < 3.5))
      && ((config(1) > 0.5) && (config(1) < 1.0))) {
    return false;
  }
  // [1.5 2.5], [1.0 4.0] locking body
  if (((config(0) > 1.5) && (config(0) < 2.5))
      && ((config(1) > 1.0) && (config(1) < 4.0))) {
    return false;
  }
    return true;
}

// Appropriately on the right and left
bool GenerateGoal(Config& v) {
  v.resize(2);
  if (Randd() < 0.5) {
    v(0) = 1.0;
    v(1) = 3.5;
  } else {
    v(0) = 3.0;
    v(1) = 3.5;
  }
  return true;
}

bool IsGoal(const Config& config) {
  // (0.9 1.1), (3.4 3.6), (3.4 3.6)
  if (((config(0) > 0.9) && (config(0) < 1.1)) &&
      ((config(1) > 3.4) && (config(1) < 3.6))) {
    return true;
  }
  // (2.9 3.1), (3.4 3.6), (3.4 3.6)
  if (((config(0) > 2.9) && (config(0) < 3.1)) &&
      ((config(1) > 3.4) && (config(1) < 3.6))) {
    return true;
  }
    return false;
}


bool IsTerminate() {
  return true;
}

}  // anonymous namespace

///////////////////////////////////////////
/// rrt_test
///////////////////////////////////////////
class RrtPlannerTest : public ::testing::Test {
 protected:
  RrtPlannerTest() {
    const auto cspace = std::make_shared<ConfigurationSpace>(kDim);
    cspace->set_random_config(RandomConfig);
    cspace->set_check_feasibility(CheckFeasibility);
    cspace->set_generate_goal_config(GenerateGoal);
    cspace->set_check_goal(IsGoal);

    planner_ = std::make_shared<RrtPlanner>(cspace, kDelta, 10000, 0.2, false);
  }
  IPointToConditionPlanner::Ptr planner_;
};

// Planning success check
TEST_F(RrtPlannerTest, plan) {
  Config init(kDim);
  init << 2.0, 0.0;

  Path path;
  ASSERT_EQ(kSuccess, planner_->PlanPath(init, path));

  // Check of PATH
  // The initial value is init
  ASSERT_DOUBLE_EQ(init(0), path.front()(0));
  ASSERT_DOUBLE_EQ(init(1), path.front()(1));

  // Goal is under conditions
  ASSERT_TRUE(IsGoal(path.back()));
  // Distance is always below Kdelta
  Config old_config = init;
  for (Path::iterator config = ++(path.begin()); config != path.end(); ++config) {
    EXPECT_TRUE((*config - old_config).norm() <= kDelta + kDoubleEps);
    EXPECT_TRUE(CheckFeasibility(*config));
    old_config = *config;
  }
}

// Ends with maximum number of repetitions
TEST(RrtPlanner, max_itr) {
  ConfigurationSpace::Ptr cspace(new ConfigurationSpace(kDim));
  cspace->set_random_config(RandomConfig);
  cspace->set_check_feasibility(CheckFeasibility);
  cspace->set_generate_goal_config(GenerateGoal);
  cspace->set_check_goal(IsGoal);

  Config init(kDim);
  init << 2.0, 0.0;

  IPointToConditionPlanner::Ptr planner(new RrtPlanner(cspace, kDelta, 10, 0.2, false));

  Path path;
  EXPECT_EQ(kMaxItr, planner->PlanPath(init, path));
}

// End with the end condition function
TEST(RrtPlanner, terminate) {
  ConfigurationSpace::Ptr cspace(new ConfigurationSpace(kDim));
  cspace->set_random_config(RandomConfig);
  cspace->set_check_feasibility(CheckFeasibility);
  cspace->set_generate_goal_config(GenerateGoal);
  cspace->set_check_goal(IsGoal);

  Config init(kDim);
  init << 2.0, 0.0;

  IPointToConditionPlanner::Ptr planner(new RrtPlanner(cspace, kDelta, 10000, 0.2, false, IsTerminate));

  Path path;
  EXPECT_EQ(kTerminate, planner->PlanPath(init, path));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
