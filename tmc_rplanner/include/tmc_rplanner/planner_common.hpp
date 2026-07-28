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
/// @file     planner_common.hpp
/// @brief    Definition of basic data structures used in the planner
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

/// Base exception class for the planner
class PlannerException : public std::exception {
 public:
  PlannerException() {}
  explicit PlannerException(const std::string& msg) : msg_(msg) {}
  virtual ~PlannerException() throw() {}
  virtual const char* what() const throw() {return msg_.c_str();}
 private:
  std::string msg_;
};

/// Exception thrown when the requested function is not found
class LackRequiredFunc : public PlannerException {
 public:
  LackRequiredFunc() {}
  explicit LackRequiredFunc(const std::string& msg) : msg_(msg) {}
  virtual ~LackRequiredFunc() throw() {}
  virtual const char* what() const throw() {return msg_.c_str();}
 private:
  std::string msg_;
};

/// Exception thrown when the dimensions of the configuration differ
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

/// Return values for tree extension
enum ExtendRet {
  kReached,   /// Reached
  kAdvanced,  /// Advanced
  kTrapped,   /// Trapped
  kFailed     /// Failed
};

/// Return values for planning
enum PlanRet {
  kSuccess,         /// Success
  kTerminate,       /// Terminated
  kMaxItr,          /// Maximum iteration count reached
  kInitConfigFail,  /// Invalid initial state
  kGoalConfigFail   /// Invalid goal state
};

/// Configuration data
using Config = Eigen::VectorXd;

/// Collision depth information
struct Collisions {
  std::string name_1;
  std::string name_2;
  double depth;

  bool IsSameObjects(const Collisions& other) const {
    return ((name_1 == other.name_1) && (name_2 == other.name_2)) ||
           ((name_1 == other.name_2) && (name_2 == other.name_1));
  }
};

/// Node for exploration
struct Node {
  using Ptr = std::shared_ptr<Node>;
  using ConstPtr = std::shared_ptr<const Node>;
  using WeakPtr = std::weak_ptr<Node>;
  using ConstWeakPtr = std::weak_ptr<const Node>;

  Node() : data(), parent() {}
  Node(const Config& d, const Node::WeakPtr& p) : data(d), parent(p) {}
  Node(const Config& d, const Node::WeakPtr& p, const std::vector<Collisions>& c) : data(d), collisions(c), parent(p) {}
  explicit Node(const Config& d) : data(d), parent() {}
  Node(const Config& d, const std::vector<Collisions>& c) : data(d), collisions(c), parent() {}
  /// Configuration
  Config data;
  /// Collision information
  std::vector<Collisions> collisions;
  /// Pointer to parent
  Node::WeakPtr parent;
};

/// Tree structure of the node
using Tree = std::deque<Node::Ptr>;
/// Path in the configuration space
using Path = std::deque<Config>;


////////// function's using = these are used in the problem /////////
/// Returns a random configuration
using RandomConfigFunc = std::function<Config ()>;
/// Configuration check
using CheckFeasibilityFunc = std::function<bool(const Config&)>;
/// Configuration check
using CheckFeasibilityWithCollisionsFunc = std::function<bool(const Config&, std::vector<Collisions>&)>;
/// Transition check between configurations
using CheckTransferabilityFunc =
    std::function<bool(const Config&, const Config&, const std::vector<Collisions>&, std::vector<Collisions>&)>;
/// Function to calculate the distance between configurations
using DistanceFunc = std::function<double(const Config&, const Config&)>;
/// Evaluation function for configurations
using EvaluateConfigFunc = std::function<double(const Config&)>;
/// Constraint condition function for configurations (subject to constraints
/// Takes a configuration and returns the constrained configuration)
using ConstraintFunc = std::function<bool(const Config&, Config&)>;
/// Check if the configuration meets the goal condition
using CheckConfigInGoalFunc = std::function<bool(const Config&)>;
/// Temporary goal creation function
using GenerateGoalConfigFunc = std::function<bool(Config&)>;
/// Temporary start creation function
using GenerateStartConfigFunc = std::function<bool(Config&)>;
/// Goal condition
using TerminateConditionFunc = std::function<bool()>;

/// Function called during configuration check, mainly for debugging
using CheckFeasibilityCallBackFunc = std::function<void(const Config&, bool)>;
/// Function called when adding a node, mainly for debugging
using AddNodeCallBackFunc = std::function<void(const Config&, const Config&)>;
/// Function called during start generation, mainly for debugging
using AddStartCallBackFunc = std::function<void(const Config&)>;
/// Function called during goal generation, mainly for debugging
using AddGoalCallBackFunc = std::function<void(const Config&)>;
/// Callback called during ConstraintConfig, mainly for debugging
using ConstrainConfigCallBackFunc = std::function<void(const Config&, const Config&, bool)>;
/// Function called during path generation
using PathCallBackFunc = std::function<void(const Path&)>;


/// @func TreeToPath
/// @brief Extract trajectory from the state tree
///        Assume the last state of the tree is the goal
///        If the path becomes larger than the size of the tree
///        Throw an exception as it is looping
/// @param tree State tree
/// @param path_out Output trajectory
/// @note If the tree has a loop, it will result in an infinite loop.
/// @exception tmc_rplanner::TreeLoop Detection of tree loop
void TreeToPath(const Tree& tree, Path& path_out);

/// @func TreeToPath
/// @brief Extract trajectory from the state tree
///        Assume the last state of the tree is the goal
///        If the path becomes larger than the size of the tree
///        Throw an exception as it is looping
/// @param tree State tree
/// @param path_out Output trajectory
/// @param goal_index Index of the final state in the state tree
/// @note If the tree has a loop, it will result in an infinite loop.
/// @exception tmc_rplanner::TreeLoop Detection of tree loop
/// @exception std::invalid_argument goal_index is larger than the size of the tree
void TreeToPath(const Tree& tree, uint32_t goal_index, Path& path_out);


/// @brief Change the root of the state tree to the specified one
/// @param tree State tree
/// @param root Node to be set as the new root
void ChangeTreeRoot(Tree& tree, const Node::WeakPtr& root);

}  // namespace tmc_rplanner

#endif  // TMC_MANIPULATION_TMC_RPLANNER_PLANNER_COMMON_HPP_
