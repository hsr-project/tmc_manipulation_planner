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
/// @file     rrt_planner.cpp
/// @brief Implementation of PointToPointPlanner using rrt_planner
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2011.10.25
/// @note     [1.0.0] 2011.10.19 Newly created

#include <ctime>
#include <stdlib.h>
#include <random>
#include <tmc_rplanner/configuration_tree.hpp>
#include <tmc_rplanner/rrt_planner.hpp>

// For debugging
// #define PRINT_TREE_

namespace tmc_rplanner {

/// @brief Extend one-step RRT
/// @param tree State space tree
/// @return true: Reached goal false: Not reached
bool RrtPlanner::BuildOneStep_(ConfigurationTree& tree) {
  Config random_config;
  bool to_goal = false;
  ExtendRet ret = kFailed;

  std::mt19937 eng(static_cast<uint32_t>(std::time(0)));
  std::uniform_real_distribution<> randf(0.0, 1.0);

  // Aim for the goal at the rate of goal_bias
  if (randf(eng) < goal_bias_) {
    space_->GenerateGoalConfig(random_config);
    to_goal = true;
  } else {
    random_config = space_->GenerateRandomConfig();
  }
  // If greedy is true, proceed towards the goal anyway
  // If greedy is false, one-step
  if (greedy_) {
    ret = tree.Connect(random_config);
  } else {
    ret = tree.Extend(random_config);
  }
  return ((ret == kReached) && (to_goal));
}

/// @brief Plan the path
/// @param inti_config Initial configuration
/// @param path_out Result path
/// @return Planning result
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
    // Check termination conditions (e.g., timeout)
    if (is_terminate_ && is_terminate_()) {
      return kTerminate;
    }
    // Extend the tree
    if (BuildOneStep_(tree)) {
      is_success = true;
      break;
    } else {
      // Check if it is a goal
      if (space_->CheckConfigInGoal(tree.GetLastConfig())) {
        is_success = true;
        break;
      }
    }
  }
  // Integrate the tree if successful
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
