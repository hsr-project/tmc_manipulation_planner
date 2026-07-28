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
/// @file     multi_birrt_planner.cpp
/// @brief Generalized BiRRT
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2012.3.27
/// @note     [1.0.0] 2012.3.28 Newly created

#include <ctime>
#include <algorithm>
#include <random>
#include <utility>
#include <vector>

#include <tmc_rplanner/configuration_tree.hpp>
#include <tmc_rplanner/multi_birrt_planner.hpp>

namespace {
using StartGoalPair = std::pair<tmc_rplanner::Config, tmc_rplanner::Config>;
using StartGoalPairSet = std::vector<StartGoalPair>;
}

namespace tmc_rplanner {

/// @brief Advance tree a one step towards a random configuration,
///        and attempt to connect tree b to tree a
/// @param space Configuration space
/// @param tree_a Exploration tree
/// @param tree_b Connection tree
/// @return true: Connection successful false: Connection failed
static bool BuildOneStepM(ConfigurationSpace::Ptr space,
                          ConfigurationTree& tree_a,
                          ConfigurationTree& tree_b,
                          TerminateConditionFunc terminate) {
  Config random_state_forward = space->GenerateRandomConfig();
  if ((tree_a.Extend(random_state_forward) == kAdvanced)
    || (tree_a.Extend(random_state_forward) == kReached)) {
    if (tree_b.Connect(tree_a.GetLastConfig(), terminate) == kReached) {
      return true;
    }
  }
  return false;
}

/// Path generation.
PlanRet MultiBirrtPlanner::PlanPath(const std::vector<Config>& start_configs,
                                    const std::vector<Config>& goal_configs,
                                    Path& path_out) {
  path_out.clear();
  std::vector<Path> paths;
  PlanRet result = PlanPaths(start_configs, goal_configs, 1, paths);
  if (!paths.empty()) {
    path_out = paths.front();
  }
  return result;
}

// Callback when a path is found
void MultiBirrtPlanner::PathCallBack_(const Path& path) {
  if (path_call_back_) {
    path_call_back_(path);
  }
}


// Path generation. Until the specified number of paths is created
PlanRet MultiBirrtPlanner::PlanPaths(const std::vector<Config>& start_configs,
                                     const std::vector<Config>& goal_configs,
                                     uint32_t max_paths,
                                     std::vector<Path>& paths_out) {
  paths_out.clear();
  StartGoalPairSet start_goal_pair_set;

  std::mt19937 eng(static_cast<uint32_t>(std::time(0)));
  std::uniform_real_distribution<> randf(0.0, 1.0);

  /* treeFromStart */
  ConfigurationTree::Ptr tree_s;
  /* treeFromGoal */
  ConfigurationTree::Ptr tree_g;
  if (max_connect_) {
    tree_s = std::make_shared<ConfigurationTree>(space_, delta_, max_connect_.get());
    tree_g = std::make_shared<ConfigurationTree>(space_, delta_, max_connect_.get());
  } else {
    tree_s = std::make_shared<ConfigurationTree>(space_, delta_);
    tree_g = std::make_shared<ConfigurationTree>(space_, delta_);
  }
  /* oneTree */
  Tree tree;
  bool is_success = false;
  bool start_config_obtained = false;
  bool goal_config_obtained = false;

  // Add after checking the initial value given as an argument
  for (std::vector<Config>::const_iterator config = start_configs.begin();
       config != start_configs.end(); ++config) {
    Config constrained_config;
    std::vector<Collisions> collisions;
    if (space_->ConstrainStartConfig(*config, constrained_config)) {
      if (space_->CheckFeasibilityWithCollisions(constrained_config, collisions)) {
        tree_s->SetRootConfig(constrained_config, collisions);
        space_->AddStartCallBack(constrained_config);
        start_config_obtained = true;
      }
    }
  }

  // Add after checking the terminal value given as an argument
  for (std::vector<Config>::const_iterator config = goal_configs.begin();
       config != goal_configs.end(); ++config) {
    Config constrained_config;
    if (space_->ConstrainGoalConfig(
            *config, constrained_config)) {
      if (space_->CheckFeasibility(constrained_config)) {
        tree_g->SetRootConfig(constrained_config);
        space_->AddGoalCallBack(constrained_config);
        goal_config_obtained = true;
      }
    }
  }

  // Main loop
  for (int32_t i = 0; i < max_itr_; ++i)  {
    is_success = false;
    // Check termination conditions (e.g., timeout)
    if (is_terminate_ && is_terminate_()) {
      if (!paths_out.empty()) {
        return kSuccess;
      } else if (!start_config_obtained) {
        return kInitConfigFail;
      } else if (!goal_config_obtained) {
        return kGoalConfigFail;
      } else {
        return kTerminate;
      }
    }

    // Add initial value with probability_start_generate probability
    // Add even if the initial value does not yet exist
    if (!start_config_obtained || randf(eng) < probability_start_generate_) {
      Config config;
      if (space_->GenerateStartConfig(config)) {
        Config constrained_config;
        std::vector<Collisions> collisions;
        if (space_->ConstrainStartConfig(config, constrained_config)) {
          if (space_->CheckFeasibilityWithCollisions(constrained_config, collisions)) {
            tree_s->SetRootConfig(constrained_config, collisions);
            space_->AddStartCallBack(constrained_config);
            start_config_obtained = true;
          }
        }
      }
    }
    if (!start_config_obtained) {
      continue;
    }

    // Add terminal value with probability_start_generate probability
    // Add even if the terminal value does not yet exist
    if (!goal_config_obtained || randf(eng) < probability_goal_generate_) {
      Config config;
      if (space_->GenerateGoalConfig(config)) {
        Config constrained_config;
        if (space_->ConstrainGoalConfig(
                config, constrained_config)) {
          if (space_->CheckFeasibility(constrained_config)) {
            tree_g->SetRootConfig(constrained_config);
            space_->AddGoalCallBack(constrained_config);
            goal_config_obtained = true;
          }
        }
      }
    }
    if (!goal_config_obtained) {
      continue;
    }

    if (i%2 == 0) {
      // from start
      if (BuildOneStepM(space_, *tree_s, *tree_g, is_terminate_)) {
        is_success = true;
      }
    } else {
      // from goal
      if (BuildOneStepM(space_, *tree_g, *tree_s, is_terminate_)) {
        is_success = true;
      }
    }
    // Add tree in case of success
    if (is_success == true) {
      Path start_path;
      Path goal_path;
      tree_s->TrackBackPath(start_path);
      tree_g->TrackBackPath(goal_path);

      // If the combination of start and goal is new
      // Add to start_goal_pair_set and add path to paths_out
      StartGoalPair new_pair(start_path.front(), goal_path.front());

      if (std::find(start_goal_pair_set.begin(),
                    start_goal_pair_set.end(),
                    new_pair) == start_goal_pair_set.end()) {
        start_goal_pair_set.push_back(new_pair);
        Path path;
        /// Paths from the goal are added in reverse order
        path = start_path;
        path.insert(path.end(), goal_path.rbegin(), goal_path.rend());
        paths_out.push_back(path);

        PathCallBack_(path);
      }
    }
    if ((paths_out.size() >= max_paths) && (max_paths != 0)) {
      break;
    }
  }
  if (!paths_out.empty()) {
    return kSuccess;
  } else if (!start_config_obtained) {
    return kInitConfigFail;
  } else if (!goal_config_obtained) {
    return kGoalConfigFail;
  } else {
    return kMaxItr;
  }
}


//  close namespace tmc_rplanner
}  // namespace tmc_rplanner
