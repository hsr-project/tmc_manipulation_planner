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
/// @brief    Definition of basic data structure used in planna


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

/// Planner basal exception class
class PlannerException : public std::exception {
 public:
  PlannerException() {}
  explicit PlannerException(const std::string& msg) : msg_(msg) {}
  virtual ~PlannerException() throw() {}
  virtual const char* what() const throw() {return msg_.c_str();}
 private:
  std::string msg_;
};

/// Exceptions issued when the request function is not found
class LackRequiredFunc : public PlannerException {
 public:
  LackRequiredFunc() {}
  explicit LackRequiredFunc(const std::string& msg) : msg_(msg) {}
  virtual ~LackRequiredFunc() throw() {}
  virtual const char* what() const throw() {return msg_.c_str();}
 private:
  std::string msg_;
};

/// Exceptions issued when the dimension of configuration is different
class DimensionMismatch : public PlannerException {
 public:
  DimensionMismatch() {}
  explicit DimensionMismatch(const std::string& msg) : msg_(msg) {}
  virtual ~DimensionMismatch() throw() {}
  virtual const char* what() const throw() {return msg_.c_str();}
 private:
  std::string msg_;
};

/// Exceptions issued when the tree is looping
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
  kReached,
  kAdvanced,
  kTrapped,
  kFailed
};

/// Return value of planning
enum PlanRet {
  kSuccess,
  kTerminate,
  kMaxItr,
  kInitConfigFail,
  kGoalConfigFail
};

/// Configuration data
using Config = Eigen::VectorXd;

/// Node for searching
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
  /// Pointer to parents
  Node::WeakPtr parent;
};

/// Node wood structure
using Tree = std::deque<Node::Ptr>;
/// Pass in the configuration space
using Path = std::deque<Config>;


////////// Function using = These are used for the problem ///////////////////////////////////////////////////////////
/// Return a random configuration
using RandomConfigFunc = std::function<Config ()>;
/// Configuration check
using CheckFeasibilityFunc = std::function<bool(const Config&)>;
/// Check transition between configurations
using CheckTransferabilityFunc = std::function<bool(const Config&, const Config&)>;
/// Distance calculation focus between configurations
using DistanceFunc = std::function<double(const Config&, const Config&)>;
/// Compatibility evaluation function
using EvaluateConfigFunc = std::function<double(const Config&)>;
/// Configuration constraints (detained)
/// Receive Configuration and return Configuration after being detained)
using ConstraintFunc = std::function<bool(const Config&, Config&)>;
/// Check if the configuration is included in the termination conditions
using CheckConfigInGoalFunc = std::function<bool(const Config&)>;
/// Temporary GOAL creation function
using GenerateGoalConfigFunc = std::function<bool(Config&)>;
/// Temporary START creation function
using GenerateStartConfigFunc = std::function<bool(Config&)>;
/// End conditions
using TerminateConditionFunc = std::function<bool()>;

/// For debugging the function of the function called when checking the configuration
using CheckFeasibilityCallBackFunc = std::function<void(const Config&, bool)>;
/// For debugging the function of the function called when node is added
using AddNodeCallBackFunc = std::function<void(const Config&, const Config&)>;
/// For debugging the function called at the time of start generation
using AddStartCallBackFunc = std::function<void(const Config&)>;
/// For debugging the function of the function called when GOAL is generated
using AddGoalCallBackFunc = std::function<void(const Config&)>;
/// For debugging mainly callbacks called at ConstraintConfig
using ConstrainConfigCallBackFunc = std::function<void(const Config&, const Config&, bool)>;
/// Functions called when Path generation
using PathCallBackFunc = std::function<void(const Path&)>;


/// @func TreeToPath
/// @brief Extract the track from the state tree
///        Suppose the end of the state tree is the goal
///        If the pass is larger than the size of TREE
///        Throw an exception because it is looped
/// @param TREE state tree
/// @param path_out Delivery
/// @note If there is a loop in Tree, it will be an infinite loop.
/// @exception TMC_RPLANNER :: TreeLoop Tree loop detection
void TreeToPath(const Tree& tree, Path& path_out);

/// @func TreeToPath
/// @brief Extract the track from the state tree
///        Suppose the end of the state tree is the goal
///        If the pass is larger than the size of TREE
///        Throw an exception because it is looped
/// @param TREE state tree
/// @param path_out Delivery
/// @param Goal_index state Index of the final state of wood
/// @note If there is a loop in Tree, it will be an infinite loop.
/// @exception TMC_RPLANNER :: TreeLoop Tree loop detection
/// @exception Std :: Invalid_argument Goal_index is larger than Tree size
void TreeToPath(const Tree& tree, uint32_t goal_index, Path& path_out);


/// @brief Change the root of the state tree to the specified one
/// @param TREE state tree
/// @param Root new Node to be rooted
void ChangeTreeRoot(Tree& tree, const Node::WeakPtr& root);

}  // namespace tmc_rplanner

#endif  // TMC_MANIPULATION_TMC_RPLANNER_PLANNER_COMMON_HPP_
