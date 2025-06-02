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
/// @file     planner_common.hpp
/// @brief    Definition of basic data structures used in planner
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2011.10.25
/// @note     [1.0.0] 2011.10.26 Newly created


#ifndef TMC_MANIPULATION_TMC_RPLANNER_PLANNER_COMMON_HPP_
#define TMC_MANIPULATION_TMC_RPLANNER_PLANNER_COMMON_HPP_

#include <stdint.h>

#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace tmc_rplanner {

/// planner base exception class
class PlannerException : public std::exception {
 public:
  PlannerException() {}
  explicit PlannerException(const std::string& msg) : msg_(msg) {}
  virtual ~PlannerException() throw() {}
  virtual const char* what() const throw() {return msg_.c_str();}
 private:
  std::string msg_;
};

/// Exception thrown when the required function is not found
class LackRequiredFunc : public PlannerException {
 public:
  LackRequiredFunc() {}
  explicit LackRequiredFunc(const std::string& msg) : msg_(msg) {}
  virtual ~LackRequiredFunc() throw() {}
  virtual const char* what() const throw() {return msg_.c_str();}
 private:
  std::string msg_;
};

/// Exception thrown when configuration dimensions differ
class DimensionMismatch : public PlannerException {
 public:
  DimensionMismatch() {}
  explicit DimensionMismatch(const std::string& msg) : msg_(msg) {}
  virtual ~DimensionMismatch() throw() {}
  virtual const char* what() const throw() {return msg_.c_str();}
 private:
  std::string msg_;
};

/// Exception thrown when the tree is looping
class TreeLoop : public PlannerException {
 public:
  TreeLoop() {}
  explicit TreeLoop(const std::string& msg) : msg_(msg) {}
  virtual ~TreeLoop() throw() {}
  virtual const char* what() const throw() {return msg_.c_str();}
 private:
  std::string msg_;
};

/// Return value of tree extension
enum ExtendRet {
  kReached,   /// Reached
  kAdvanced,  /// Extended
  kTrapped,   /// Unable to extend
  kFailed     /// Failed
};

/// Return value of planning
enum PlanRet {
  kSuccess,         /// Success
  kTerminate,       /// Forced termination
  kMaxItr,          /// Maximum iteration count reached
  kInitConfigFail,  /// Invalid initial state
  kGoalConfigFail   /// Invalid goal state
};

/// Configuration data
using Config = Eigen::VectorXd;

/// Node for exploration
struct Node {
  using Ptr = std::shared_ptr<Node>;
  using ConstPtr = std::shared_ptr<const Node>;
  using WeakPtr = std::weak_ptr<Node>;
  using ConstWeakPtr = std::weak_ptr<const Node>;

  Node() : data(), parent() {}
  Node(const Config& d, const Node::WeakPtr& p) : data(d), parent(p) {}
  explicit Node(const Config& d) : data(d), parent() {}
  /// Configuration
  Config data;
  /// Pointer to parent
  Node::WeakPtr parent;
};

/// Tree structure of node
using Tree = std::deque<Node::Ptr>;
/// Path in configuration space
using Path = std::deque<Config>;


////////// using function = these are used for the problem /////////
/// Returns a random configuration
using RandomConfigFunc = std::function<Config ()>;
/// Configuration check
using CheckFeasibilityFunc = std::function<bool(const Config&)>;
/// Transition check between configurations
using CheckTransferabilityFunc = std::function<bool(const Config&, const Config&)>;
/// Function for calculating distance between configurations
using DistanceFunc = std::function<double(const Config&, const Config&)>;
/// Evaluation function of configuration
using EvaluateConfigFunc = std::function<double(const Config&)>;
/// Configuration constraint condition function (subject to constraints
/// Takes configuration, returns constrained configuration)
using ConstraintFunc = std::function<bool(const Config&, Config&)>;
/// Check if configuration is in termination condition
using CheckConfigInGoalFunc = std::function<bool(const Config&)>;
/// Temporary goal creation function
using GenerateGoalConfigFunc = std::function<bool(Config&)>;
/// Temporary start creation function
using GenerateStartConfigFunc = std::function<bool(Config&)>;
/// Termination condition
using TerminateConditionFunc = std::function<bool()>;

/// Function called during configuration check Mainly for debugging
using CheckFeasibilityCallBackFunc = std::function<void(const Config&, bool)>;
/// Function called during node addition Mainly for debugging
using AddNodeCallBackFunc = std::function<void(const Config&, const Config&)>;
/// Function called during start generation Mainly for debugging
using AddStartCallBackFunc = std::function<void(const Config&)>;
/// Function called during goal generation Mainly for debugging
using AddGoalCallBackFunc = std::function<void(const Config&)>;
/// Callback called during ConstraintConfig Mainly for debugging
using ConstrainConfigCallBackFunc = std::function<void(const Config&, const Config&, bool)>;
/// Function called during path generation
using PathCallBackFunc = std::function<void(const Path&)>;


/// @func TreeToPath
/// @brief Extract trajectory from state tree
///        Assume end of state tree is goal
///        If path becomes larger than tree size
///        Looping occurs, so throw exception
/// @param tree State tree
/// @param path_out Output trajectory
/// @note Loop in tree leads to infinite loop.
/// @exception tmc_rplanner::TreeLoop Detect tree loop
void TreeToPath(const Tree& tree, Path& path_out);

/// @func TreeToPath
/// @brief Extract trajectory from state tree
///        Assume end of state tree is goal
///        If path becomes larger than tree size
///        Looping occurs, so throw exception
/// @param tree State tree
/// @param path_out Output trajectory
/// @param goal_index Index of final state in state tree
/// @note Loop in tree leads to infinite loop.
/// @exception tmc_rplanner::TreeLoop Detect tree loop
/// @exception std::invalid_argument goal_index larger than tree size
void TreeToPath(const Tree& tree, uint32_t goal_index, Path& path_out);


/// @brief Change state tree root to specified one
/// @param tree State tree
/// @param root Node to become new root
void ChangeTreeRoot(Tree& tree, const Node::WeakPtr& root);

}  // namespace tmc_rplanner

#endif  // TMC_MANIPULATION_TMC_RPLANNER_PLANNER_COMMON_HPP_
