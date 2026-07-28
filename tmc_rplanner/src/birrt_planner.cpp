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
/// @file     birrt_planner.cpp
/// @brief Implementation of PointToPointPlanner using birrt_planner
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2011.10.25
/// @note     [1.0.0] 2011.10.19 Newly created

#include <tmc_rplanner/birrt_planner.hpp>
#include <tmc_rplanner/configuration_tree.hpp>

// For debugging
// #define PRINT_TREE_

namespace tmc_rplanner {

/// @brief Advance tree_a one step towards a random configuration,
///        and attempt to connect tree_b to tree_a
/// @param space Configuration space
/// @param tree_a Exploration tree
/// @param tree_b Connection tree
/// @return true: Connection successful false: Connection failed
bool BuildOneStep(ConfigurationSpace::Ptr space,
                  ConfigurationTree& tree_a,
                  ConfigurationTree& tree_b) {
  Config random_state_forward = space->GenerateRandomConfig();
  if ((tree_a.Extend(random_state_forward) == kAdvanced)
      || (tree_a.Extend(random_state_forward) == kReached)) {
    if (tree_b.Connect(tree_a.GetLastConfig()) == kReached) {
      return true;
    }
  }
  return false;
}

/// @brief Path generation
/// @param init_config Initial configuration
/// @param goal_config Goal configuration
/// @param path_out Generated trajectory
/// @return kSucess: Success
/// @return kTerminate: Termination
/// @return kMaxItr: Maximum iteration count reached
PlanRet BiRrtPlanner::PlanPath(const Config& init_config,
                        const Config& goal_config,
                        Path& path_out) {
  path_out.clear();
  /* treeFromStart */
  ConfigurationTree tree_s(space_, delta_);
  /* treeFromGoal */
  ConfigurationTree tree_g(space_, delta_);
  /* oneTree */
  Tree tree;
  bool is_success = false;
  // Initial collisions are allowed with the expectation of escaping, but collisions at the goal are not allowed
  std::vector<Collisions> init_collisions;
  if (!space_->CheckFeasibilityWithCollisions(init_config, init_collisions)) {
    return kInitConfigFail;
  }
  if (!space_->CheckFeasibility(goal_config)) {
    return kGoalConfigFail;
  }
  tree_s.SetRootConfig(init_config, init_collisions);
  tree_g.SetRootConfig(goal_config);
  for (int32_t i = 0; i < max_itr_; ++i)  {
    // Check termination conditions (e.g., timeout)
    if (is_terminate_ && is_terminate_()) {
      return kTerminate;
    }
    // from start
    if (i%2 == 0) {
      if (BuildOneStep(space_, tree_s, tree_g)) {
        is_success = true;
        break;
      }
    } else {     // from goal
      if (BuildOneStep(space_, tree_g, tree_s)) {
        is_success = true;
        break;
      }
    }
  }
  // Merge trees in case of success
  if (is_success == true) {
    Path start_path;
    Path goal_path;
    tree_s.TrackBackPath(start_path);
    tree_g.TrackBackPath(goal_path);

#ifdef PRINT_TREE_
    tree_s.PrintTree(std::cerr);
    tree_g.PrintTree(std::cerr);
#endif
    /// Paths from the goal are added in reverse order
    path_out = start_path;
    path_out.insert(path_out.end(), goal_path.rbegin(), goal_path.rend());
    return kSuccess;
  } else {
    return kMaxItr;
  }
}

//  close namespace tmc_rplanner
}  // namespace tmc_rplanner
