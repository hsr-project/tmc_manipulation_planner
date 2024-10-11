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
/// @brief    ConfigurationSpace test

#include <stdlib.h>
#include <gtest/gtest.h>
#include <tmc_rplanner/configuration_space.hpp>

using tmc_rplanner::CheckTransferabilityByDividing;
using tmc_rplanner::Config;
using tmc_rplanner::ConfigurationSpace;
using tmc_rplanner::DimensionMismatch;
using tmc_rplanner::DistanceFunc;
using tmc_rplanner::LackRequiredFunc;
using tmc_rplanner::Node;
using tmc_rplanner::Tree;
using tmc_rplanner::TreeLoop;
using tmc_rplanner::Path;

namespace {
// State space used in the test
int32_t kDim = 2;
// A threshold that is regarded as the value of DOUBLE is near
double kDoubleEps = 1.0e-5;

static double Randd() {
  static uint32_t seed = 1;
  return static_cast<double>(rand_r(&seed)) / RAND_MAX;
}

// Random function for testing
Config RandomConfig() {
  Config v(kDim);
  v(0) = Randd()*4.0;
  v(1) = Randd()*4.0;
  return v;
}

// Configuration check for testing
bool CheckConfig(const Config& config) {
  if (((config(0) < 3.5) && (config(0) > 0)) &&
      ((config(1) < 1.5) && (config(1) > 1.0))) return false;
  if (((config(0) < 4.0) && (config(0) > 0.5)) &&
      ((config(1) < 3.5) && (config(1) > 3.0))) return false;
  return true;
}

// Test configuration interpretation transition
bool CheckTrans(const Config& src_config, const Config& dst_config) {
  return CheckTransferabilityByDividing(
      src_config, dst_config, CheckConfig, DistanceFunc(), 0.01);
}

// Check function for checktransferabilitybydivingTest
// (0,0), (1,1), (1,1), (0.4-0.6], [0.4-0.6])
bool TestDividingFunc(const Config& config) {
  return !(((config(0) > 0.4) && (config(0) < 0.6))
           && ((config(1) > 0.4) && (config(1) < 0.6)));
}

// Distance of distance weight between test configurations
double CalcDistance(const Config& src_config, const Config& dst_config) {
  return sqrt(1.0 * pow(src_config(0) - dst_config(0), 2.0) +
              1.0 * pow(src_config(1) - dst_config(1), 2.0));
}

// Functions that are the maximum in test configuration evaluation (0,0)
double EvalConfig(const Config& config) {
  return exp(-pow((config).norm(), 2));
}

// Test restraint condition function
bool ConstraintConfig(const Config& config_in, Config& config_out) {
  config_out = config_in;
  config_out(0) = 0.0;
  return true;
}


// Start for testing for testing
bool ConstraintStartConfig(const Config& config_in, Config& config_out) {
  config_out = config_in;
  config_out(0) = 1.0;
  return true;
}

// Test goal restraint condition function
bool ConstraintGoalConfig(const Config& config_in, Config& config_out) {
  config_out = config_in;
  config_out(0) = 2.0;
  return true;
}

// Test goal judgment function
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

// Testing goal creation function
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

// Testing start creation function
bool GenerateStart(Config& v) {
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
}  // anonymous namespace

///////////////////////////////////////////
/// CHECKTRANSFERABITYBYDIDIDING test
///////////////////////////////////////////


// Test to be finely divided and checked properly
TEST(CheckTransferabilityByDividingTest, normal_test) {
  int32_t dim = kDim;
  Config src_config(dim);
  src_config << 0.0, 0.0;
  Config dst_config(dim);
  dst_config << 1.0, 1.0;
  double sub_delta = 0.1;
  EXPECT_FALSE(CheckTransferabilityByDividing(
      src_config, dst_config, TestDividingFunc,
      DistanceFunc(), sub_delta));
}

// If the dimension of SRC_CONFIG and DST_CONFIG is different, tmc_rplanner :: dimensionMatch
TEST(CheckTransferabilityByDividingTest, dim_mismatch) {
  int32_t dim_src = kDim;
  int32_t dim_dst = kDim + 1;
  Config src_config(dim_src);
  src_config << 0.0, 0.0;
  Config dst_config(dim_dst);
  dst_config << 1.0, 1.0, 1.0;
  double sub_delta = 0.1;
  EXPECT_THROW(
      CheckTransferabilityByDividing(
          src_config,
          dst_config,
          TestDividingFunc,
          DistanceFunc(),
          sub_delta),
      DimensionMismatch);
}

// If sub_delta is negative Std :: Invalid_argument
TEST(CheckTransferabilityByDividingTest, negative_subdelta) {
  int32_t dim_src = kDim;
  int32_t dim_dst = kDim;
  Config src_config(dim_src);
  src_config << 0.0, 0.0;
  Config dst_config(dim_dst);
  dst_config << 1.0, 1.0;
  double sub_delta = -0.1;
  EXPECT_THROW(
    CheckTransferabilityByDividing(
        src_config, dst_config, TestDividingFunc,
        DistanceFunc(), sub_delta), std::invalid_argument);
}

///////////////////////////////////
/// TreeTopath test
//////////////////////////////////

class TreeToPathTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    config1_.resize(kDim);
    config2_.resize(kDim);
    config3_.resize(kDim);
    config4_.resize(kDim);
    config1_ << 0.0, 0.0;
    config2_ << 0.1, 0.1;
    config3_ << 0.2, 0.2;
    config4_ << 0.3, 0.3;

    // Make Tree
    // 1->2->3
    //     ->4
    tree_.push_back(Node::Ptr(new Node(config1_)));
    tree_.push_back(Node::Ptr(new Node(config2_, tree_.back())));
    tree_.push_back(Node::Ptr(new Node(config3_, tree_.back())));
    tree_.push_back(Node::Ptr(new Node(config4_, tree_[1])));
  }
  Tree tree_;
  Config config1_;
  Config config2_;
  Config config3_;
  Config config4_;
};

// Check if you can convert correctly with TreeTopath
TEST_F(TreeToPathTest, translate_path_last) {
  Path path;
  // The goal is the final element of the tree
  TreeToPath(tree_, path);
  ASSERT_EQ(3, path.size());
  EXPECT_EQ(config1_, path[0]);
  EXPECT_EQ(config2_, path[1]);
  EXPECT_EQ(config4_, path[2]);
}

// Check if you can convert correctly with TreeTopath
TEST_F(TreeToPathTest, translate_path_mid) {
  Path path;
  // The goal is the intermediate element of the tree
  TreeToPath(tree_, 2, path);
  ASSERT_EQ(3, path.size());
  EXPECT_EQ(config1_, path[0]);
  EXPECT_EQ(config2_, path[1]);
  EXPECT_EQ(config3_, path[2]);
}

// Check the loop on TreeTopath
TEST_F(TreeToPathTest, translate_path_loop) {
  Path path;
  tree_[1]->parent = tree_[3];
  // The goal is the final element of the tree
  EXPECT_THROW(TreeToPath(tree_, path), TreeLoop);
}

// Checking the wrong Goal_index in TreeTopath
TEST_F(TreeToPathTest, translate_path_invalid_goal) {
  Path path;
  // The goal is the final element of the tree
  EXPECT_THROW(TreeToPath(tree_, 5, path),
               std::invalid_argument);
}


///////////////////////////////////
/// Changetreeroot test
//////////////////////////////////

class ChangeTreeRootTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    config0_.resize(kDim);
    config1_.resize(kDim);
    config2_.resize(kDim);
    config3_.resize(kDim);
    config0_ << 0.0, 0.0;
    config1_ << 0.1, 0.1;
    config2_ << 0.2, 0.2;
    config3_ << 0.3, 0.3;

    // Make Tree
    // 0->1->2
    //     ->3
    tree_.push_back(Node::Ptr(new Node(config0_)));
    tree_.push_back(Node::Ptr(new Node(config1_, tree_.back())));
    tree_.push_back(Node::Ptr(new Node(config2_, tree_.back())));
    tree_.push_back(Node::Ptr(new Node(config3_, tree_[1])));
  }
  Tree tree_;
  Config config0_;
  Config config1_;
  Config config2_;
  Config config3_;
};

// Try Changetreeroot
TEST_F(ChangeTreeRootTest, change_tree_root3) {
  ChangeTreeRoot(tree_, tree_[3]);
  EXPECT_TRUE(tree_[3]->parent.expired());
  EXPECT_TRUE(tree_[2]->parent.lock() == tree_[1]);
  EXPECT_TRUE(tree_[1]->parent.lock() == tree_[3]);
  EXPECT_TRUE(tree_[0]->parent.lock() == tree_[1]);
}

// Try Changetreeroot
TEST_F(ChangeTreeRootTest, change_tree_root2) {
  ChangeTreeRoot(tree_, tree_[2]);
  EXPECT_TRUE(tree_[3]->parent.lock() == tree_[1]);
  EXPECT_TRUE(tree_[2]->parent.expired());
  EXPECT_TRUE(tree_[1]->parent.lock() == tree_[2]);
  EXPECT_TRUE(tree_[0]->parent.lock() == tree_[1]);
}

// Try Changetreeroot
TEST_F(ChangeTreeRootTest, change_tree_root1) {
  ChangeTreeRoot(tree_, tree_[1]);
  EXPECT_TRUE(tree_[3]->parent.lock() == tree_[1]);
  EXPECT_TRUE(tree_[2]->parent.lock() == tree_[1]);
  EXPECT_TRUE(tree_[1]->parent.expired());
  EXPECT_TRUE(tree_[0]->parent.lock() == tree_[1]);
}

// Try Changetreeroot
TEST_F(ChangeTreeRootTest, change_tree_root0) {
  ChangeTreeRoot(tree_, tree_[0]);
  EXPECT_TRUE(tree_[3]->parent.lock() == tree_[1]);
  EXPECT_TRUE(tree_[2]->parent.lock() == tree_[1]);
  EXPECT_TRUE(tree_[1]->parent.lock() == tree_[0]);
  EXPECT_TRUE(tree_[0]->parent.expired());
}




///////////////////////////////////
/// Configuration_space test
//////////////////////////////////

// Is the set function called normal?
class ConfigurationSpaceTest : public ::testing::Test {
 protected:
  ConfigurationSpaceTest() : space_(kDim) {}
  virtual void SetUp() {
    space_.set_random_config(RandomConfig);
    space_.set_check_feasibility(CheckConfig);
    space_.set_check_transferability(CheckTrans);
    space_.set_distance(CalcDistance);
    space_.set_evaluate_config(EvalConfig);
    space_.set_check_goal(IsGoal);
    space_.set_generate_start_config(GenerateStart);
    space_.set_generate_goal_config(GenerateGoal);
    space_.set_constrain_config(ConstraintConfig);
    space_.set_constrain_start_config(ConstraintStartConfig);
    space_.set_constrain_goal_config(ConstraintGoalConfig);
  }
  ConfigurationSpace space_;
};

// Is the set function called normal?
class ConfigurationSpaceTestNoConstrain : public ::testing::Test {
 protected:
  ConfigurationSpaceTestNoConstrain() : space_(kDim) {}
  virtual void SetUp() {
    space_.set_random_config(RandomConfig);
    space_.set_check_feasibility(CheckConfig);
    space_.set_check_transferability(CheckTrans);
    space_.set_distance(CalcDistance);
    space_.set_evaluate_config(EvalConfig);
    space_.set_check_goal(IsGoal);
    space_.set_generate_goal_config(GenerateGoal);
    space_.set_generate_start_config(GenerateStart);
  }
  ConfigurationSpace space_;
};


// For testing with the function is not set
class ConfigurationSpaceTestNoFunctions : public ::testing::Test {
 protected:
  ConfigurationSpaceTestNoFunctions() : space_(kDim) {}
  ConfigurationSpace space_;
};

// NEWCONFIG test
TEST_F(ConfigurationSpaceTest, new_config) {
  Config src_config(kDim);
  src_config(0) = 0.0;
  src_config(1) = 0.0;
  Config dst_config(kDim);
  dst_config(0) = 1.0;
  dst_config(1) = 1.0;

  // Check the configuration that goes on one step
  bool unreach;
  Config next_config = space_.NewConfig(src_config, dst_config, 0.1, unreach);
  EXPECT_FALSE(unreach);
  EXPECT_NEAR(0.070711, next_config(0), kDoubleEps);
  EXPECT_NEAR(0.070711, next_config(1), kDoubleEps);

  // Check when arriving
  Config dst_reach_config(kDim);
  dst_reach_config(0) = 0.05;
  dst_reach_config(1) = 0.0;
  bool reach;
  Config reach_config = space_.NewConfig(
      src_config, dst_reach_config, 0.1, reach);
  EXPECT_TRUE(reach);
  EXPECT_NEAR(dst_reach_config(0), reach_config(0), kDoubleEps);
  EXPECT_NEAR(dst_reach_config(1), reach_config(1), kDoubleEps);

  // Exception DELTA is negative
  EXPECT_THROW(
      space_.NewConfig(src_config, dst_config, -0.1, unreach),
      std::invalid_argument);

  // Exception DOF difference
  Config inval_config(kDim+1);
  inval_config(0) = 0.0;
  inval_config(1) = 0.0;
  inval_config(2) = 0.0;
  EXPECT_THROW(
      space_.NewConfig(inval_config, dst_config, 0.1, unreach),
      DimensionMismatch);
}

// Random configuration generation function
TEST_F(ConfigurationSpaceTest, random_config) {
  Config random_config = space_.GenerateRandomConfig();
  EXPECT_TRUE((random_config(0) >= 0.0 && random_config(0) <= 4.0));
  EXPECT_TRUE((random_config(1) >= 0.0 && random_config(1) <= 4.0));
}

// When checkline test constrain is fraudulent
TEST_F(ConfigurationSpaceTest, check_line) {
  Config src_config(kDim);
  src_config(0) = 0.0;
  src_config(1) = 0.0;
  Config dst_config(kDim);
  dst_config(0) = 1.0;
  dst_config(1) = 1.0;
  Path path;
  // It should be imagined in the same place and fails
  EXPECT_FALSE(space_.CheckLine(src_config, dst_config, 0.1, path));
}

// Exceptions because there is no random configuration function function function
TEST_F(ConfigurationSpaceTestNoFunctions, no_random_config_func) {
  EXPECT_THROW(
      space_.GenerateRandomConfig(),
      LackRequiredFunc);
}

// CHECKLINE test
TEST_F(ConfigurationSpaceTestNoConstrain, check_line) {
  Config src_config(kDim);
  src_config(0) = 0.0;
  src_config(1) = 0.0;
  Config dst_config(kDim);
  dst_config(0) = 1.0;
  dst_config(1) = 1.0;
  Path path;

  // Example of passing
  EXPECT_TRUE(space_.CheckLine(src_config, dst_config, 0.1, path));
  // There are initial values ​​and ends
  EXPECT_TRUE(path.front() == src_config);
  EXPECT_TRUE(path.back() == dst_config);

  // Config is very close
  dst_config(0) = 0.0;
  dst_config(1) = 1.0e-7;
  EXPECT_TRUE(space_.CheckLine(src_config, dst_config, 0.1, path));
  EXPECT_NEAR(dst_config(1), path.back()(1), 1.0e-8);

  // An example of a configuration that is not a FEASIBLE on the way
  Config infeasible_dst_config(kDim);
  infeasible_dst_config(0) = 1.0;
  infeasible_dst_config(1) = 1.2;
  Path infeasible_path;
  EXPECT_FALSE(space_.CheckLine(
      src_config, infeasible_dst_config, 0.1, infeasible_path));

  // Exception DOF difference
  Config inval_config(kDim+1);
  inval_config(0) = 0.0;
  inval_config(1) = 0.0;
  inval_config(2) = 0.0;

  EXPECT_THROW(
    space_.CheckLine(inval_config, dst_config, 0.1, infeasible_path),
    DimensionMismatch);
}

// Check for Feasty
TEST_F(ConfigurationSpaceTest, check_feasibility) {
  Config feasible_config(kDim);
  feasible_config << 0.0, 0.0;
  Config infeasible_config(kDim);
  infeasible_config << 1.0, 1.2;
  EXPECT_TRUE(space_.CheckFeasibility(feasible_config));
  EXPECT_FALSE(space_.CheckFeasibility(infeasible_config));

  // Exception DOF difference
  Config inval_config(kDim+1);
  inval_config(0) = 0.0;
  inval_config(1) = 0.0;
  inval_config(2) = 0.0;
  EXPECT_THROW(
      space_.CheckFeasibility(inval_config),
      DimensionMismatch);
}

// The exception is that the feasibility function is not set
TEST_F(ConfigurationSpaceTestNoFunctions, no_check_feasibility_func) {
  Config feasible_config(kDim);
  feasible_config << 0.0, 0.0;
  EXPECT_THROW(
      space_.CheckFeasibility(feasible_config),
      LackRequiredFunc);
}

// Check of checktransferability
TEST_F(ConfigurationSpaceTest, check_transferability) {
  Config src_config(kDim);
  src_config << 0.0, 0.0;
  Config feasible_config(kDim);
  feasible_config << 1.0, 1.0;
  Config infeasible_config(kDim);
  infeasible_config << 0.1, 2.0;
  // Transitionable example
  EXPECT_TRUE(space_.CheckTransferability(src_config, feasible_config));
  // Unable to transition
  EXPECT_FALSE(space_.CheckTransferability(src_config, infeasible_config));

  // Exception DOF difference
  Config inval_config(kDim+1);
  inval_config(0) = 0.0;
  inval_config(1) = 0.0;
  inval_config(2) = 0.0;

  EXPECT_THROW(
      space_.CheckTransferability(inval_config, infeasible_config),
      DimensionMismatch);
}

// Operation correctly even if Transferability is not set
TEST_F(ConfigurationSpaceTestNoFunctions, no_check_transferability_func) {
  space_.set_check_feasibility(CheckConfig);
  Config src_config(kDim);
  src_config << 0.0, 0.0;
  Config feasible_config(kDim);
  feasible_config << 1.0, 1.0;
  Config infeasible_config(kDim);
  infeasible_config << 0.1, 1.2;
  // Transitionable example
  EXPECT_TRUE(space_.CheckTransferability(src_config, feasible_config));
  // Unable to transition
  EXPECT_FALSE(space_.CheckTransferability(src_config, infeasible_config));
}


// Distance measurement
TEST_F(ConfigurationSpaceTest, calc_distance) {
  Config src_config(kDim);
  src_config(0) = 0.0;
  src_config(1) = 0.0;
  Config dst_config(kDim);
  dst_config(0) = 1.0;
  dst_config(1) = 1.0;
  EXPECT_NEAR(
      sqrt(2.0), space_.CalcDistance(src_config, dst_config), kDoubleEps);
}

// The distance measurement Euglid distance is automatically used.
TEST_F(ConfigurationSpaceTestNoFunctions, no_calc_distance_func) {
  Config src_config(kDim);
  src_config(0) = 0.0;
  src_config(1) = 0.0;
  Config dst_config(kDim);
  dst_config(0) = 1.0;
  dst_config(1) = 1.0;
  EXPECT_NEAR(
      sqrt(2.0), space_.CalcDistance(src_config, dst_config), kDoubleEps);
}

// Assessment of configuration
TEST_F(ConfigurationSpaceTest, eval_func) {
  Config config(kDim);
  config(0) = 0.0;
  config(1) = 0.0;
  EXPECT_NEAR(1.0, space_.EvaluateConfig(config), kDoubleEps);
  // Exception DOF difference
  Config inval_config(kDim+1);
  inval_config(0) = 0.0;
  inval_config(1) = 0.0;
  inval_config(2) = 0.0;
  EXPECT_THROW(
      space_.EvaluateConfig(inval_config),
      DimensionMismatch);
}

// Exceptions because the EVALUATE function is not set
TEST_F(ConfigurationSpaceTestNoFunctions, no_evaluate_config_func) {
  Config config(kDim);
  config(0) = 0.0;
  config(1) = 0.0;
  EXPECT_THROW(
      space_.EvaluateConfig(config),
      LackRequiredFunc);
}


// GOAL configuration generation
TEST_F(ConfigurationSpaceTest, goal_func) {
  Config goal;
  space_.GenerateGoalConfig(goal);
  EXPECT_TRUE((goal(0) == 1.0 && goal(1) == 3.5) ||
              (goal(0) == 3.0 && goal(1) == 3.5));
}

// Exceptions because the Goal generated function is not set
TEST_F(ConfigurationSpaceTestNoFunctions, no_generate_goal_config_func) {
  Config a;
  EXPECT_FALSE(space_.GenerateGoalConfig(a));
}

// Generation of START configuration
TEST_F(ConfigurationSpaceTest, start_func) {
  Config start;
  space_.GenerateStartConfig(start);
  EXPECT_TRUE((start(0) == 1.0 && start(1) == 3.5) ||
              (start(0) == 3.0 && start(1) == 3.5));
}

// Exceptions because the Goal generated function is not set
TEST_F(ConfigurationSpaceTestNoFunctions, no_generate_start_config_func) {
  Config a;
  EXPECT_FALSE(space_.GenerateStartConfig(a));
}


// Goal condition judgment test
TEST_F(ConfigurationSpaceTest, is_goal_func) {
  Config goal_config(kDim);
  goal_config << 1.0, 3.5;
  Config non_goal_config(kDim);
  non_goal_config << 0.0, 0.0;
  EXPECT_TRUE(space_.CheckConfigInGoal(goal_config));
  EXPECT_FALSE(space_.CheckConfigInGoal(non_goal_config));

  // Exception DOF difference
  Config inval_config(kDim+1);
  inval_config(0) = 0.0;
  inval_config(1) = 0.0;
  inval_config(2) = 0.0;
  EXPECT_THROW(
      space_.CheckConfigInGoal(inval_config),
      DimensionMismatch);
}


// Exceptions because the Goal generated function is not set
TEST_F(ConfigurationSpaceTestNoFunctions, no_is_goal_config_func) {
  Config goal_config(kDim);
  goal_config << 1.0, 3.5;
  EXPECT_THROW(
      space_.CheckConfigInGoal(goal_config),
      LackRequiredFunc);
}


// Restraint the configuration
TEST_F(ConfigurationSpaceTest, constraint_config_func) {
  Config config(kDim);
  config << 1.0, 3.5;
  Config constrainted_config(kDim);
  EXPECT_TRUE(space_.ConstrainConfig(config, constrainted_config));
  EXPECT_NEAR(0.0, constrainted_config(0), kDoubleEps);

  // Exception DOF difference
  Config inval_config(kDim+1);
  inval_config(0) = 0.0;
  inval_config(1) = 0.0;
  inval_config(2) = 0.0;
  EXPECT_THROW(
      space_.ConstrainConfig(inval_config, constrainted_config),
      DimensionMismatch);
}


// Success without any restraint generation functions
TEST_F(ConfigurationSpaceTestNoFunctions, no_constraint_config_func) {
  Config config(kDim);
  Config constrainted_config(kDim);
  config << 1.0, 3.5;
  EXPECT_TRUE(space_.ConstrainConfig(config, constrainted_config));
  EXPECT_TRUE(config == constrainted_config);
}


// Restraint the configuration
TEST_F(ConfigurationSpaceTest, constraint_start_config_func) {
  Config config(kDim);
  config << 0.0, 3.5;
  Config constrainted_config(kDim);
  EXPECT_TRUE(space_.ConstrainStartConfig(config, constrainted_config));
  EXPECT_NEAR(1.0, constrainted_config(0), kDoubleEps);

  // Exception DOF difference
  Config inval_config(kDim+1);
  inval_config(0) = 0.0;
  inval_config(1) = 0.0;
  inval_config(2) = 0.0;
  EXPECT_THROW(
      space_.ConstrainStartConfig(inval_config, constrainted_config),
      DimensionMismatch);
}


// Restraint the configuration
TEST_F(ConfigurationSpaceTest, constraint_goal_config_func) {
  Config config(kDim);
  config << 1.0, 3.5;
  Config constrainted_config(kDim);
  EXPECT_TRUE(space_.ConstrainGoalConfig(config, constrainted_config));
  EXPECT_NEAR(2.0, constrainted_config(0), kDoubleEps);

  // Exception DOF difference
  Config inval_config(kDim+1);
  inval_config(0) = 0.0;
  inval_config(1) = 0.0;
  inval_config(2) = 0.0;
  EXPECT_THROW(
      space_.ConstrainGoalConfig(inval_config, constrainted_config),
      DimensionMismatch);
}



// Success without any restraint generation functions
TEST_F(ConfigurationSpaceTestNoFunctions, no_constraint_start_config_func) {
  Config config(kDim);
  Config constrainted_config(kDim);
  config << 1.0, 3.5;
  EXPECT_TRUE(space_.ConstrainStartConfig(config, constrainted_config));
  EXPECT_TRUE(config == constrainted_config);
}


// Success without any restraint generation functions
TEST_F(ConfigurationSpaceTestNoFunctions, no_constraint_goal_config_func) {
  Config config(kDim);
  Config constrainted_config(kDim);
  config << 1.0, 3.5;
  EXPECT_TRUE(space_.ConstrainGoalConfig(config, constrainted_config));
  EXPECT_TRUE(config == constrainted_config);
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
