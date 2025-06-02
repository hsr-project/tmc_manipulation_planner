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
/// @file     configuration_tree.hpp
/// @brief    Tree structure class in configuration space used in planner
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2011.11.01
/// @note     [1.0.0] 2011.10.26 Newly created

#ifndef TMC_MANIPULATION_TMC_RPLANNER_CONFIGURATION_TREEHPP_
#define TMC_MANIPULATION_TMC_RPLANNER_CONFIGURATION_TREEHPP_

#include <memory>
#include <tmc_rplanner/configuration_space.hpp>
#include <tmc_rplanner/planner_common.hpp>

namespace tmc_rplanner {

class ConfigurationTree {
 public:
  using Ptr = std::shared_ptr<ConfigurationTree>;
  using ConstPtr = std::shared_ptr<const ConfigurationTree>;

  ConfigurationTree(ConfigurationSpace::Ptr configuration_space, double delta);
  ConfigurationTree(ConfigurationSpace::Ptr configuration_space, double delta, int32_t max_connect);
  ~ConfigurationTree() {}

  /// Extend branches from the nearest node of the tree to dst_config by a distance of delta
  /// @param[in] dst_config Target configuration
  ExtendRet Extend(const Config& dst_config);

  /// Perform Extend until reaching dst_config
  /// @param[in] dst_config Target configuration
  ExtendRet Connect(const Config& dst_config);

  /// Perform Extend until reaching dst_config
  /// @param[in] dst_config Target configuration
  /// @param[in] terminate Termination function
  ExtendRet Connect(const Config& dst_config,
                    TerminateConditionFunc terminate);
  /// Initialize the tree
  void ClearTree() {tree_.clear();}
  /// Output the tree to a stream
  void PrintTree() const;
  /// Extract path with the final element as the goal
  void TrackBackPath(Path& path_out) const;

  /// Get the latest configuration
  /// @return Latest configuration
  Config GetLastConfig() const {return tree_.back()->data;}

  /// Remove the branch connected to the latest configuration
  void RemoveLastBranch();

  /// Set the root configuration
  /// @param[in] config The configuration to set as root
  void SetRootConfig(const Config& config) {
    tree_.push_front(Node::Ptr(new Node(config)));
  }

  /// Get the current number of nodes
  /// @return Current number of nodes
  int32_t GetNumNode() const {
    return tree_.size();
  }

  double delta() const {return delta_;}
  void set_delta(double delta) {delta_ = delta;}

 private:
  /// Copying is prohibited
  ConfigurationTree(const ConfigurationTree&);
  ConfigurationTree& operator=(const ConfigurationTree&);

  /// Configuration tree
  Tree tree_;
  /// Configuration space
  ConfigurationSpace::Ptr configuration_space_;
  /// Exploration width of the tree
  double delta_;
  /// Maximum number of times to continue Extend in Connect; if negative, continue as much as possible
  int32_t max_connect_;
  /// Retrieve the nearest node within the tree
  Node::WeakPtr FetchNearestNeighbor_(const Config& config);
};
}  // namespace tmc_rplanner

#endif  // TMC_MANIPULATION_TMC_RPLANNER_CONFIGURATION_TREEHPP_
