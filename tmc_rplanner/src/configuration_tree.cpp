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
/// @file     configuration_tree.cpp
/// @brief    Configuration space used in the planner
///           Class summarizing tree structure and operations
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2011.10.25
/// @note     [1.0.0] 2011.10.19 Newly created

#include <float.h>
#include <iostream>
#include <tmc_rplanner/configuration_tree.hpp>

namespace {
// Maximum continuation of Extend
int32_t kMaxConnect = 10;
}

namespace tmc_rplanner {


ConfigurationTree::ConfigurationTree(ConfigurationSpace::Ptr configuration_space, double delta) :
      configuration_space_(configuration_space), delta_(delta), max_connect_(kMaxConnect) {}

ConfigurationTree::ConfigurationTree(ConfigurationSpace::Ptr configuration_space, double delta, int32_t max_connect) :
      configuration_space_(configuration_space), delta_(delta), max_connect_(max_connect) {}

/// @func Extend
/// @brief Add a new state to the tree
/// @param dst_config Target state
/// @param tree State tree
/// @retval kReached Reached the target state
/// @retval kAdvanced Approached the target state
/// @retval kTrapped Unable to approach the target state
ExtendRet ConfigurationTree::Extend(const Config& dst_config) {
  // Ensure the size of Configuration is correct
  if (configuration_space_->dof() != dst_config.size()) {
    throw DimensionMismatch("Configuration size mismatch.");
  }
  Node::WeakPtr nearest = FetchNearestNeighbor_(dst_config);
  bool is_reached(false);
  Config new_config = configuration_space_->
      NewConfig(nearest.lock()->data, dst_config, delta_, is_reached);
  Config next_config = new_config;
  if (!configuration_space_->ConstrainConfig(new_config, next_config)) {
    return kTrapped;
  }
  // Prevent exceeding delta_ or more
  bool is_constrained_reached(false);
  next_config = configuration_space_->
      NewConfig(nearest.lock()->data, next_config,
                delta_, is_constrained_reached);

  ExtendRet ret = kFailed;
  std::vector<Collisions> next_collisions;
  if (configuration_space_->CheckTransferability(nearest.lock()->data,
                                                 next_config,
                                                 nearest.lock()->collisions,
                                                 next_collisions)) {
    Node::Ptr next_node(new Node(next_config, nearest, next_collisions));
    tree_.push_back(next_node);
    configuration_space_->AddNodeCallBack(nearest.lock()->data, next_config);
    if (is_reached) {
      ret = kReached;
    } else {
      // Return trapped if nearest is closer, otherwise Advanced
      if ((nearest.lock()->data - dst_config).norm() <
          (next_config - dst_config).norm()) {
        return kTrapped;
      }
      return kAdvanced;
    }
  } else {
    return kTrapped;
  }
  return ret;
}



/// @func Connect
/// @brief Continue Extend until reaching the specified state from the tree
/// @param dst_config Target state
/// @param tree State tree
/// @retval kReached Reached the target state
/// @retval kAdvanced Approached the target state
/// @retval kTrapped Unable to approach the target state
ExtendRet ConfigurationTree::Connect(const Config& dst_config) {
  return Connect(dst_config, TerminateConditionFunc());
}


/// @func Connect
/// @brief Continue Extend until reaching the specified state from the tree
///        However, with termination conditions
/// @param dst_config Target state
/// @param terminate Termination condition function
/// @retval kReached Reached the target state
/// @retval kAdvanced Approached the target state
/// @retval kTrapped Unable to approach the target state
ExtendRet ConfigurationTree::Connect(const Config& dst_config,
                                     TerminateConditionFunc terminate) {
  if (max_connect_ == 0) {
    Node::WeakPtr nearest = FetchNearestNeighbor_(dst_config);
    std::vector<Collisions> dst_collisions;
    if (configuration_space_->CalcDistance(nearest.lock()->data, dst_config) < delta_ &&
        configuration_space_->CheckTransferability(nearest.lock()->data,
                                                   dst_config,
                                                   nearest.lock()->collisions,
                                                   dst_collisions)) {
      return kReached;
    } else {
      return kFailed;
    }
  }
  ExtendRet ret = kAdvanced;
  int32_t i = 0;
  while ((ret == kAdvanced) && ((max_connect_ < 0) || (i < max_connect_))) {
    ret = Extend(dst_config);
    if (terminate && terminate()) {
      return kFailed;
    }
    ++i;
  }
  return ret;
}

/// @func FetchNearestNeighbor_
/// @brief Retrieve the nearest neighbor from the tree
/// @param tree State tree
/// @param config State to find the nearest neighbor for
/// @retval Nearest node
Node::WeakPtr ConfigurationTree::FetchNearestNeighbor_(const Config& config) {
  double min = DBL_MAX;
  Node::WeakPtr nearest_node;
  for (Tree::iterator node = tree_.begin(); node != tree_.end(); ++node) {
    double distance = configuration_space_->CalcDistance((*node)->data, config);
    if (distance < min) {
      min = distance;
      nearest_node = *node;
    }
  }
  return nearest_node;
}

/// @func PrintTree
/// @brief Output the tree to Storm, mainly for debugging
void ConfigurationTree::PrintTree() const {
  for (Tree::const_iterator node = tree_.begin(); node != tree_.end(); ++node) {
    if (!(*node)->parent.expired()) {
      std::cout << (*node)->data.transpose() << " "
                << (*node)->parent.lock()->data.transpose() << std::endl;
    } else {
      std::cout << (*node)->data.transpose() << " Root" << std::endl;
    }
  }
}

/// @func TrackBackPath
/// @brief Retrieve the path from the tree
/// @param path_out Output path
void ConfigurationTree::TrackBackPath(Path& path_out) const {
  TreeToPath(tree_, path_out);
}

/// @func RemoveLastBranch
/// @brief Remove the branch connected to the latest configuration
void ConfigurationTree::RemoveLastBranch() {
  if (tree_.empty()) {
    return;
  }
  auto root_node_num = tree_.size();
  for (auto it = tree_.begin(); it != tree_.end(); ++it) {
    if (!(*it)->parent.expired()) {
      root_node_num = std::distance(tree_.begin(), it);
      break;
    }
  }
  // If all are Root, the leftmost one is the last added Root, so just delete it and finish
  if (root_node_num == tree_.size()) {
    tree_.pop_front();
    return;
  }

  Node::WeakPtr node = tree_.back();
  while (!node.lock()->parent.expired()) {
    node = node.lock()->parent;
  }
  for (auto it = tree_.begin(); it != tree_.end(); ++it) {
    if (*it == node.lock()) {
      tree_.erase(it);
      break;
    }
  }
  // tree_ adds RootNode from the left and extended Nodes from the right
  // Therefore, there will always be child Nodes on the right side of the deleted one, so deleting them sequentially will remove all
  for (auto it = tree_.begin() + root_node_num - 1; it != tree_.end(); ) {
    if ((*it)->parent.expired()) {
      it = tree_.erase(it);
    } else {
      ++it;
    }
  }
}

// end of namespace tmc_planner
}  // namespace tmc_rplanner
