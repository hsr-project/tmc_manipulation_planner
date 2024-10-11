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
/// @brief    Tree class in a configuration space used in planna

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

  /// Recently extend the branch from the distance of DELTA to DST_CONFIG from nearby TREE
  /// @param[in] dst_config Goal configuration
  ExtendRet Extend(const Config& dst_config);

  /// Do the Extend until you reach DST_CONFIG
  /// @param[in] dst_config Goal configuration
  ExtendRet Connect(const Config& dst_config);

  /// Do the Extend until you reach DST_CONFIG
  /// @param[in] dst_config Goal configuration
  /// @param[in] terminate Termination function
  ExtendRet Connect(const Config& dst_config,
                    TerminateConditionFunc terminate);
  /// Initialize the tree
  void ClearTree() {tree_.clear();}
  /// Output the tree into the stream
  void PrintTree() const;
  /// GOAL is the final element that extracts the path
  void TrackBackPath(Path& path_out) const;

  /// Acquisition of the latest configuration
  /// @return The latest configuration
  Config GetLastConfig() const {return tree_.back()->data;}

  /// Delete branches connected to the latest configuration
  void RemoveLastBranch();

  /// Set root configuration
  /// @param[in] config Root configuration
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
  /// Copies are prohibited
  ConfigurationTree(const ConfigurationTree&);
  ConfigurationTree& operator=(const ConfigurationTree&);

  /// Configuration tree
  Tree tree_;
  /// Configuration space
  ConfigurationSpace::Ptr configuration_space_;
  /// Tree search width
  double delta_;
  /// Continue as much as the maximum number of times and negative values ​​to continue Extended with Connect.
  int32_t max_connect_;
  /// Get a recent node from TREE
  Node::WeakPtr FetchNearestNeighbor_(const Config& config);
};
}  // namespace tmc_rplanner

#endif  // TMC_MANIPULATION_TMC_RPLANNER_CONFIGURATION_TREEHPP_
