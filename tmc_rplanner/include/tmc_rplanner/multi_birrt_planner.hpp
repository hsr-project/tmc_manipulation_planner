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
/// @brief    Birrt has been extended to be added to the initial value, the terminal value of multiple and added.

#ifndef TMC_MANIPULATION_TMC_RPLANNER_MULTI_BIRRT_PLANNER_HPP_
#define TMC_MANIPULATION_TMC_RPLANNER_MULTI_BIRRT_PLANNER_HPP_

#include <vector>
#include <boost/optional.hpp>
#include <tmc_rplanner/configuration_space.hpp>
#include <tmc_rplanner/multi_planner.hpp>

namespace tmc_rplanner {

/// Multibirrtplanner search parameter
struct MultiBirrtPlannerParam {
  /// Exploration
  double delta;
  /// Maximum number of repeated times
  int32_t max_itr;
  /// Initial configuration generation probability [0.0 ~ 1.0]
  double probability_start_generate;
  /// End configuration generation probability [0.0 ~ 1.0]
  double probability_goal_generate;
  /// End condition function
  TerminateConditionFunc is_terminate;
  /// Maximum number of Connect operation
  boost::optional<int32_t> max_connect;
};

/// @class MultiBirrtPlanner
/// @brief Different from the plunner Birlt with Birlt extended in the following points
///        1. Has multiple initial and terminal values.However, with the Generate function
///          If it is added, 0 may be.
///        2. Initial values ​​with the Generate function defined in ConfigurationSpace
///          And the terminal value is Probavility_start_generate,
///          And the probability_goal_generate probability is added.
///        Birrt is upward compatible and basically you can use this.
class MultiBirrtPlanner : public IMultiPlanner {
 public:
  /// @brief Pass the planner space and the termination conditions
  /// @param space Configuration space
  /// @param Param Search parameter
  MultiBirrtPlanner(const ConfigurationSpace::Ptr space,
                    const MultiBirrtPlannerParam& param) :
      space_(space), delta_(param.delta), max_itr_(param.max_itr),
      probability_start_generate_(param.probability_start_generate),
      probability_goal_generate_(param.probability_goal_generate),
      max_connect_(param.max_connect), is_terminate_(param.is_terminate) {}

  virtual ~MultiBirrtPlanner() {}

  /// Path generation.
  /// @param start_configs Initial configuration set
  /// @param goal_configs Goal configuration set
  /// @param path_out Generated path
  virtual PlanRet PlanPath(const std::vector<Config>& start_configs,
                           const std::vector<Config>& goal_configs,
                           Path& path_out);

  /// Production of paths. Until the number of passes is available
  /// @param start_configs Initial configuration set
  /// @param goal_configs Goal configuration set
  /// @param max_paths Maximum number of paths
  /// @param paths_out Generated paths
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

  /// Callback called when a path is generated
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
