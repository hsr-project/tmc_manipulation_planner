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
/// @file     multi_birrt_planner.hpp
/// @brief    Extended version of birrt for use with multiple and added initial and terminal values.
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2012.03.28
/// @note     [1.0.0] 2012.03.28 Newly created

#ifndef TMC_MANIPULATION_TMC_RPLANNER_MULTI_BIRRT_PLANNER_HPP_
#define TMC_MANIPULATION_TMC_RPLANNER_MULTI_BIRRT_PLANNER_HPP_

#include <vector>
#include <boost/optional.hpp>
#include <tmc_rplanner/configuration_space.hpp>
#include <tmc_rplanner/multi_planner.hpp>

namespace tmc_rplanner {

/// Exploration parameters for MultiBirrtPlanner
struct MultiBirrtPlannerParam {
  /// Exploration width
  double delta;
  /// Maximum number of iterations
  int32_t max_itr;
  /// Probability of initial configuration generation [0.0~1.0]
  double probability_start_generate;
  /// Probability of goal configuration generation [0.0~1.0]
  double probability_goal_generate;
  /// Termination condition function
  TerminateConditionFunc is_terminate;
  /// Maximum number of Connect actions
  boost::optional<int32_t> max_connect;
};

/// @class MultiBirrtPlanner
/// @brief Planner extended from BiRRT differs from birrt in the following points
///        1. Possesses multiple initial and terminal values. However, in the generate function,
///          zero is acceptable in case of additions.
///        2. Initial and terminal values are added at the probabilities of
///           probability_start_generate and probability_goal_generate respectively,
///           by the generate function defined in ConfigurationSpace.
///        It is a backward-compatible version of BiRRT and should generally be used.
class MultiBirrtPlanner : public IMultiPlanner {
 public:
  /// @brief Pass the planner space and the end condition.
  /// @param space Pointer to the planner space
  /// @param param Search parameters
  MultiBirrtPlanner(const ConfigurationSpace::Ptr space,
                    const MultiBirrtPlannerParam& param) :
      space_(space), delta_(param.delta), max_itr_(param.max_itr),
      probability_start_generate_(param.probability_start_generate),
      probability_goal_generate_(param.probability_goal_generate),
      max_connect_(param.max_connect), is_terminate_(param.is_terminate) {}

  virtual ~MultiBirrtPlanner() {}

  /// Path generation.
  /// @param start_configs Collection of initial configurations (not required)
  /// @param goal_configs Collection of goal configurations (not required)
  /// @param path_out Generated path
  /// @retval kSuccess: Success
  /// @retval kTerminate: Termination
  /// @retval kInitConfigFail: No feasible initial value and generation also impossible
  /// @retval kGoalConfigFail: No feasible goal value and generation also impossible
  /// @retval kMaxItr: Maximum number of repetitions reached/// Path creation
  virtual PlanRet PlanPath(const std::vector<Config>& start_configs,
                           const std::vector<Config>& goal_configs,
                           Path& path_out);

  /// Path generation. Until the specified number of paths is created
  /// @param[in] start_configs Collection of initial configurations (not required)
  /// @param[in] goal_configs Collection of goal configurations (not required)
  /// @param[in] max_paths Finishes when this number of paths are created. 0 for indefinite
  /// @param[out] paths_out Generated paths (multiple)
  /// @retval kSuccess: Success
  /// @retval kTerminate: Termination
  /// @retval kInitConfigFail: No feasible initial value and generation also impossible
  /// @retval kGoalConfigFail: No feasible goal value and generation also impossible
  /// @retval kMaxItr: Maximum number of repetitions reached
  virtual PlanRet PlanPaths(const std::vector<Config>& start_configs,
                            const std::vector<Config>& goal_configs,
                            uint32_t max_paths,
                            std::vector<Path>& paths_out);

  virtual void set_path_call_back(PathCallBackFunc call_back) {
    path_call_back_ = call_back;
  }

 private:
  // Copy prohibition
  MultiBirrtPlanner(const MultiBirrtPlanner&);
  MultiBirrtPlanner& operator=(const MultiBirrtPlanner&);

  /// Callback called when the path is generated
  /// @param[in] path Passes the generated path to the callback
  /// Behavior: Calls the callback set by SetPathCallBack.
  /// Does nothing if not set.
  virtual void PathCallBack_(const Path& path);

  const ConfigurationSpace::Ptr space_;
  const double delta_;
  const int32_t max_itr_;
  const double probability_start_generate_;
  const double probability_goal_generate_;
  const boost::optional<int32_t> max_connect_;
  PathCallBackFunc path_call_back_;
  TerminateConditionFunc is_terminate_;
};
}  // namespace tmc_rplanner

#endif
