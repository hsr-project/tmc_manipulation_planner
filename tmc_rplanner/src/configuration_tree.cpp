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
/// @brief    In the configuration space used in the planna
///           Classes that summarize wood structure and operation

#include <float.h>
#include <iostream>
#include <tmc_rplanner/configuration_tree.hpp>

namespace {
// Maximum to continue EXTEND
int32_t kMaxConnect = 10;
}

namespace tmc_rplanner {


ConfigurationTree::ConfigurationTree(ConfigurationSpace::Ptr configuration_space, double delta) :
      configuration_space_(configuration_space), delta_(delta), max_connect_(kMaxConnect) {}

ConfigurationTree::ConfigurationTree(ConfigurationSpace::Ptr configuration_space, double delta, int32_t max_connect) :
      configuration_space_(configuration_space), delta_(delta), max_connect_(max_connect) {}

/// @func Extend
ExtendRet ConfigurationTree::Extend(const Config& dst_config) {
  // Configuration size is correct
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
  // Delta_ Don't let it be above
  bool is_constrained_reached(false);
  next_config = configuration_space_->
      NewConfig(nearest.lock()->data, next_config,
                delta_, is_constrained_reached);

  ExtendRet ret = kFailed;
  if (configuration_space_->CheckTransferability(nearest.lock()->data,
                                                 next_config)) {
    Node::Ptr next_node(new Node(next_config, nearest));
    tree_.push_back(next_node);
    configuration_space_->AddNodeCallBack(nearest.lock()->data, next_config);
    if (is_reached) {
      ret = kReached;
    } else {
      // If Nearest is closer, Advensed if you don't return Trapped
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
/// @brief Continue Extend until you reach the specified state from the tree
ExtendRet ConfigurationTree::Connect(const Config& dst_config) {
  return Connect(dst_config, TerminateConditionFunc());
}


/// @func Connect
/// @brief Continue Extend until you reach the specified state from the tree
///        However, with the end conditions
ExtendRet ConfigurationTree::Connect(const Config& dst_config,
                                     TerminateConditionFunc terminate) {
  if (max_connect_ == 0) {
    Node::WeakPtr nearest = FetchNearestNeighbor_(dst_config);
    if (configuration_space_->CalcDistance(nearest.lock()->data, dst_config) < delta_ &&
        configuration_space_->CheckTransferability(nearest.lock()->data, dst_config)) {
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
/// @brief Recently acquired nearby from the tree
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
/// @brief Output the tree to the storm.Mainly for Debug
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
/// @brief Get a pass from the tree
void ConfigurationTree::TrackBackPath(Path& path_out) const {
  TreeToPath(tree_, path_out);
}

/// @func RemoveLastBranch
/// @brief Delete branches connected to the latest configuration
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
  // In the case of root, the leftmost is the last added, so erase it and end it.
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
  // Tree_ adds rootnode from the left and the extended node from the right
  // Therefore, there is a child node on the right side of the thing that has been erased, so if you erase it in order, you can erase it all.
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
