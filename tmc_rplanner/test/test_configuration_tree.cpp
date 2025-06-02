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
/// @file     test_configuration_tree.cpp
/// @brief    Test of ConfigurationTree
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2011.11.30
/// @note     [1.0.0] 2011.11.30 Newly created

#include <stdlib.h>
#include <gtest/gtest.h>
#include <tmc_rplanner/configuration_tree.hpp>

using tmc_rplanner::CheckTransferabilityByDividing;
using tmc_rplanner::Config;
using tmc_rplanner::ConfigurationSpace;
using tmc_rplanner::ConfigurationTree;
using tmc_rplanner::DimensionMismatch;
using tmc_rplanner::DistanceFunc;
using tmc_rplanner::kAdvanced;
using tmc_rplanner::kReached;
using tmc_rplanner::kTrapped;
using tmc_rplanner::kFailed;

namespace {
// Dimension of the state space used in the test
int32_t kDim = 2;


static double Randd() {
  static uint32_t seed = 1;
  return static_cast<double>(rand_r(&seed))/RAND_MAX;
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

// Configuration transition for testing
bool CheckTrans(const Config& src_config, const Config& dst_config) {
  return CheckTransferabilityByDividing(
      src_config, dst_config, CheckConfig, DistanceFunc(), 0.01);
}

// Check function for CheckTransferabilityByDividingTest
// Function that returns true for (0,0), (1,1) and false for ([0.4~0.6],[0.4~0.6])
bool TestDividingFunc(const Config& config) {
  return (!(((config(0) > 0.4) && (config(0) < 0.6))
            && ((config(1) > 0.4) && (config(1) < 0.6))));
}

// Distance between configurations for testing Weighted distance
double CalcDistance(const Config& src_config, const Config& dst_config) {
  return sqrt(1.0 * pow(src_config(0) - dst_config(0), 2.0) +
              1.0 * pow(src_config(1) - dst_config(1), 2.0));
}
}  // anonymous namespace

///////////////////////////////////////////
/// ConfigurationTree
///////////////////////////////////////////
class ConfigurationTreeTest : public ::testing::Test {
 protected:
  ConfigurationTreeTest() {
    const auto cspace = std::make_shared<ConfigurationSpace>(kDim);
    cspace->set_random_config(RandomConfig);
    cspace->set_check_feasibility(CheckConfig);
    cspace->set_check_transferability(CheckTrans);
    cspace->set_distance(CalcDistance);

    ctree_ = std::make_shared<ConfigurationTree>(cspace, 0.2);
  }

  void ResetTree(int32_t max_connect) {
    const auto cspace = std::make_shared<ConfigurationSpace>(kDim);
    cspace->set_random_config(RandomConfig);
    cspace->set_check_feasibility(CheckConfig);

    ctree_ = std::make_shared<ConfigurationTree>(cspace, 0.2, max_connect);

    Config start(kDim);
    start(0) = 0.0;
    start(1) = 0.0;
    ctree_->SetRootConfig(start);
  }

  ConfigurationTree::Ptr ctree_;
};

TEST_F(ConfigurationTreeTest, Extend) {
  Config start(kDim);
  start(0) = 0.0;
  start(1) = 0.0;
  ctree_->SetRootConfig(start);

  // Extend towards 4,4.
  Config goal(kDim);
  goal(0) = 4.0;
  goal(1) = 4.0;
  EXPECT_EQ(kAdvanced, ctree_->Extend(goal));
  EXPECT_EQ(2, ctree_->GetNumNode());

  EXPECT_EQ(kAdvanced, ctree_->Extend(goal));
  EXPECT_EQ(3, ctree_->GetNumNode());

  EXPECT_EQ(kAdvanced, ctree_->Extend(goal));
  EXPECT_EQ(4, ctree_->GetNumNode());

  EXPECT_EQ(kAdvanced, ctree_->Extend(goal));
  EXPECT_EQ(5, ctree_->GetNumNode());

  EXPECT_EQ(kAdvanced, ctree_->Extend(goal));
  EXPECT_EQ(6, ctree_->GetNumNode());

  EXPECT_EQ(kAdvanced, ctree_->Extend(goal));
  EXPECT_EQ(7, ctree_->GetNumNode());

  EXPECT_EQ(kAdvanced, ctree_->Extend(goal));
  EXPECT_EQ(8, ctree_->GetNumNode());

  // Should trap here
  EXPECT_EQ(kTrapped, ctree_->Extend(goal));
  EXPECT_EQ(8, ctree_->GetNumNode());

  // Extend towards 0.5,0.4.
  Config new_goal(kDim);
  new_goal(0) = 0.5;
  new_goal(1) = 0.4;
  // This should be Reached
  EXPECT_EQ(kReached, ctree_->Extend(new_goal));
  EXPECT_EQ(9, ctree_->GetNumNode());

  // Dimension error Exception
  Config inval_goal(kDim+1);
  inval_goal(0) = 0.5;
  inval_goal(1) = 0.4;
  inval_goal(2) = 0.4;
  EXPECT_THROW(ctree_->Extend(inval_goal), DimensionMismatch);
}

TEST_F(ConfigurationTreeTest, Connect) {
  Config start(kDim);
  start(0) = 0.0;
  start(1) = 0.0;
  ctree_->SetRootConfig(start);

  // Extend towards 3.5,0.5. Does not reach
  Config goal(kDim);
  goal(0) = 3.5;
  goal(1) = 0.5;
  EXPECT_EQ(kAdvanced, ctree_->Connect(goal));

  // Extend towards 4.0, 4.0. Should not reach.
  Config new_goal(kDim);
  new_goal(0) = 4.0;
  new_goal(1) = 4.0;
  EXPECT_EQ(kTrapped, ctree_->Connect(new_goal));

  // Dimension error Exception
  Config inval_goal(kDim+1);
  inval_goal(0) = 0.5;
  inval_goal(1) = 0.4;
  inval_goal(2) = 0.4;
  EXPECT_THROW(ctree_->Connect(inval_goal), DimensionMismatch);
}

TEST_F(ConfigurationTreeTest, SetMaxConnect) {
  // Progress in increments of 0.2 from 3.0, should reach at 15 but not at 14
  ResetTree(15);

  Config goal(kDim);
  goal(0) = 3.0;
  goal(1) = 0.0;
  EXPECT_EQ(kReached, ctree_->Connect(goal));

  ResetTree(14);
  EXPECT_EQ(kAdvanced, ctree_->Connect(goal));

  // In case of negative numbers, attempt to extend as much as possible
  ResetTree(-1);
  EXPECT_EQ(kReached, ctree_->Connect(goal));

  // In case of 0, check only if it is possible to connect without extending the tree
  ResetTree(0);

  // If the tree has been extended, it should be Reached on the second attempt, so perform twice to check if it has not been extended
  goal(0) = 0.21;
  EXPECT_EQ(kFailed, ctree_->Connect(goal));
  EXPECT_EQ(kFailed, ctree_->Connect(goal));

  goal(0) = 0.19;
  EXPECT_EQ(kReached, ctree_->Connect(goal));
}

TEST_F(ConfigurationTreeTest, RemoveLastBranch) {
  // Build the tree, the index and value of each Node should be as follows
  // 3 (0.0, 0.0) - 4 (0.0, 0.2) - 7 (0.0, 0.4)
  // 2 (1.0, 0.0) - 5 (1.0, 0.2) - 6 (1.0, 0.4)
  //                |- 8 (1.2, 0.2)
  // 1 (2.0, 0.0)
  // 0 (3.0, 0.0)
  Config start(kDim);
  start(0) = 0.0;
  start(1) = 0.0;
  ctree_->SetRootConfig(start);

  start(0) = 1.0;
  start(1) = 0.0;
  ctree_->SetRootConfig(start);

  start(0) = 2.0;
  start(1) = 0.0;
  ctree_->SetRootConfig(start);

  start(0) = 3.0;
  start(1) = 0.0;
  ctree_->SetRootConfig(start);

  Config goal(kDim);
  goal(0) = 0.0;
  goal(1) = 1.0;
  EXPECT_EQ(kAdvanced, ctree_->Extend(goal));

  goal(0) = 1.0;
  goal(1) = 1.0;
  EXPECT_EQ(kAdvanced, ctree_->Extend(goal));
  EXPECT_EQ(kAdvanced, ctree_->Extend(goal));

  goal(0) = 0.0;
  goal(1) = 1.0;
  EXPECT_EQ(kAdvanced, ctree_->Extend(goal));

  goal(0) = 1.3;
  goal(1) = 0.2;
  EXPECT_EQ(kAdvanced, ctree_->Extend(goal));

  EXPECT_EQ(9, ctree_->GetNumNode());

  // All branches extended from 2 should disappear
  ctree_->RemoveLastBranch();
  EXPECT_EQ(5, ctree_->GetNumNode());

  // All branches extended from 3 should disappear
  ctree_->RemoveLastBranch();
  EXPECT_EQ(2, ctree_->GetNumNode());

  // 0 should disappear and 1 should remain
  ctree_->RemoveLastBranch();
  EXPECT_EQ(1, ctree_->GetNumNode());
  EXPECT_DOUBLE_EQ(2.0, ctree_->GetLastConfig()[0]);

  // 1 disappears leaving it empty
  ctree_->RemoveLastBranch();
  EXPECT_EQ(0, ctree_->GetNumNode());

  // No exception occurs even when attempting to delete in an empty state
  ctree_->RemoveLastBranch();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
