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
/// @brief    Implementation of PointOpointplanner by rrt_planner

#include <ctime>
#include <stdlib.h>
#include <random>
#include <tmc_rplanner/configuration_tree.hpp>
#include <tmc_rplanner/rrt_planner.hpp>

// For debugging
// #define PRINT_TREE_

namespace tmc_rplanner {

/// @brief 1 Step RRT extends
bool RrtPlanner::BuildOneStep_(ConfigurationTree& tree) {
  Config random_config;
  bool to_goal = false;
  ExtendRet ret = kFailed;

  std::mt19937 eng(static_cast<uint32_t>(std::time(0)));
  std::uniform_real_distribution<> randf(0.0, 1.0);

  // Aim for the goal at the ratio of Goal_bias
  if (randf(eng) < goal_bias_) {
    space_->GenerateGoalConfig(random_config);
    to_goal = true;
  } else {
    random_config = space_->GenerateRandomConfig();
  }
  // If GREEDY is true, go to Goal anyway
  // If GREEDY is False, 1-STEP
  if (greedy_) {
    ret = tree.Connect(random_config);
  } else {
    ret = tree.Extend(random_config);
  }
  return ((ret == kReached) && (to_goal));
}

/// @brief Plan a path
PlanRet RrtPlanner::PlanPath(const Config& init_config,
                             Path& path_out) {
  path_out.clear();
  /* treeFromStart */
  ConfigurationTree tree(space_, delta_);

  bool is_success = false;
  if (!space_->CheckFeasibility(init_config)) {
    return kInitConfigFail;
  }
  tree.SetRootConfig(init_config);
  for (int32_t i = 0; i < max_itr_; ++i)  {
    // Check the end conditions (timeout, etc.)
    if (is_terminate_ && is_terminate_()) {
      return kTerminate;
    }
    // Expand Tree
    if (BuildOneStep_(tree)) {
      is_success = true;
      break;
    } else {
      // GOAL Judgment
      if (space_->CheckConfigInGoal(tree.GetLastConfig())) {
        is_success = true;
        break;
      }
    }
  }
  // Integrate the tree in the case of success
  if (is_success == true) {
    Path path;
    tree.TrackBackPath(path_out);

#ifdef PRINT_TREE_
    tree.PrintTree(std::cerr);
#endif
    return kSuccess;
  } else {
    return kMaxItr;
  }
}

//  close namespace tmc_rplanner
}  // namespace tmc_rplanner
