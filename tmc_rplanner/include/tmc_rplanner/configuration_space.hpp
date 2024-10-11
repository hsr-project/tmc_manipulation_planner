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
/// @brief    Configuration space basic operation for planning

#ifndef TMC_MANIPULATION_TMC_RPLANNER_CONFIGURATION_SPACE_HPP_
#define TMC_MANIPULATION_TMC_RPLANNER_CONFIGURATION_SPACE_HPP_

#include <stdint.h>
#include <memory>

#include <tmc_rplanner/planner_common.hpp>

namespace tmc_rplanner {

/// @brief Most basic checktransferabilityFunc
bool CheckTransferabilityByDividing(
     const Config& src_config,
     const Config& dst_config,
     const CheckFeasibilityFunc& check_feasibility,
     const DistanceFunc& calc_distance,
     double sub_delta);

/// @brief State space definition
class ConfigurationSpace {
 public:
  using Ptr = std::shared_ptr<ConfigurationSpace>;
  using ConstPtr = std::shared_ptr<const ConfigurationSpace>;

  explicit ConfigurationSpace(int32_t dof) : dof_(dof) {}
  ~ConfigurationSpace() {}

  int32_t dof() const {return dof_;}

  void set_random_config(RandomConfigFunc generate_random_config) {
    generate_random_config_ = generate_random_config;
  }
  void set_check_feasibility(CheckFeasibilityFunc check_feasibility) {
    check_feasibility_ = check_feasibility;
  }
  void set_check_transferability(
      CheckTransferabilityFunc check_transferability) {
    check_transferability_ = check_transferability;
  }
  void set_distance(DistanceFunc calc_distance) {
    calc_distance_ = calc_distance;
  }
  void set_evaluate_config(EvaluateConfigFunc evaluate_config) {
    evaluate_config_ = evaluate_config;
  }
  void set_check_goal(CheckConfigInGoalFunc check_goal_config) {
    check_goal_config_ = check_goal_config;
  }
  void set_generate_goal_config(GenerateGoalConfigFunc generate_goal_config) {
    generate_goal_config_ = generate_goal_config;
  }
  void set_generate_start_config(GenerateGoalConfigFunc generate_start_config) {
    generate_start_config_ = generate_start_config;
  }
  void set_constrain_config(ConstraintFunc constrain_config) {
    constrain_config_ = constrain_config;
  }
  void set_constrain_start_config(ConstraintFunc constrain_start_config) {
    constrain_start_config_ = constrain_start_config;
  }
  void set_constrain_goal_config(ConstraintFunc constrain_goal_config) {
    constrain_goal_config_ = constrain_goal_config;
  }

  void set_check_feasibility_callback(
      CheckFeasibilityCallBackFunc check_feasibility_callback) {
    check_feasibility_callback_ = check_feasibility_callback;
  }

  void set_add_node_callback(
      AddNodeCallBackFunc add_node_callback) {
    add_node_callback_ = add_node_callback;
  }

  void set_add_start_callback(
      AddStartCallBackFunc add_start_callback) {
    add_start_callback_ = add_start_callback;
  }

  void set_add_goal_callback(
      AddGoalCallBackFunc add_goal_callback) {
    add_goal_callback_ = add_goal_callback;
  }

  void set_constrain_config_callback(
      ConstrainConfigCallBackFunc constrain_config_callback) {
    constrain_config_callback_ = constrain_config_callback;
  }

  /// Return one step from SRC_CONFIG to DST_CONFIG
  Config NewConfig(const Config& src_config, const Config& dst_config,
                   double delta,  bool& is_reached_out) const;
  /// Check a straight line
  bool CheckLine(const Config& src_conifg, const Config& dst_config,
                 double delta, Path& path_out) const;
  /// Check the straight line (with Termite)
  bool CheckLine(const Config& src_conifg, const Config& dst_config,
                 double delta, TerminateConditionFunc terminate,
                 Path& path_out) const;
  /// Configuration is valid
  bool CheckFeasibility(const Config& config) const;
  /// Transition potential check Required: check_fease or check_transferability
  bool CheckTransferability(const Config& src_config,
                            const Config& dst_config) const;
  /// Random configuration generation Required: Random_config
  Config GenerateRandomConfig() const;
  /// Measure the distance between configurations Optional: Distance
  double CalcDistance(const Config& config1, const Config& config2) const;
  /// Configuration evaluation Required: EVALUATE_CONFIG
  double EvaluateConfig(const Config& config) const;
  /// Goal generation function Required: Generate_goal_config
  bool GenerateGoalConfig(Config& config) const;
  /// Start generation function Required: Generate_start_config
  bool GenerateStartConfig(Config& config) const;
  /// Check if the configuration is included in the termination conditions
  /// required: check_goal_config
  bool CheckConfigInGoal(const Config& config) const;
  /// Required: Constrain_config to restrain configuration
  bool ConstrainConfig(const Config& config_in, Config& config_out) const;
  /// Required: constrain_config_goal to restrain the configuration of START
  bool ConstrainStartConfig(const Config& config_in, Config& config_out) const;
  /// REQUIRED: constrain_config_goal to restrain Goal configuration
  bool ConstrainGoalConfig(const Config& config_in, Config& config_out) const;
  /// For debugging the function of the function called when checking the configuration
  void CheckFeasibilityCallBack(const Config& config,  bool success) const;
  /// For debugging the function of the function called when node is added
  void AddNodeCallBack(const Config& parent, const Config& child) const;
  /// For debugging the function called at the time of start generation
  void AddStartCallBack(const Config& config) const;
  /// For debugging the function of the function called when GOAL is generated
  void AddGoalCallBack(const Config& config) const;
  /// For debugging mainly callbacks called at ConstraintConfig
  void ConstrainConfigCallBack(const Config& config_in,
                               const Config& config_out, bool success) const;

 private:
  // Copies are prohibited
  ConfigurationSpace(const ConfigurationSpace&);
  ConfigurationSpace& operator = (const ConfigurationSpace&);
  /// Status space freedom
  const int32_t dof_;
  /// A function that returns a rander state
  RandomConfigFunc generate_random_config_;
  /// A function that returns whether the current configuration is possible
  CheckFeasibilityFunc check_feasibility_;
  /// Check if you can transition two configurations
  CheckTransferabilityFunc check_transferability_;
  /// Distance between two configurations
  DistanceFunc calc_distance_;
  /// Compatibility evaluation function
  EvaluateConfigFunc evaluate_config_;
  /// Check if the configuration is included in the termination conditions
  CheckConfigInGoalFunc check_goal_config_;
  /// Configuration restraint function
  ConstraintFunc constrain_config_;
  /// Start configuration restraint function
  ConstraintFunc constrain_start_config_;
  /// GOAL configuration restraint function
  ConstraintFunc constrain_goal_config_;
  /// START creation function
  GenerateStartConfigFunc generate_start_config_;
  /// Goal creation function
  GenerateGoalConfigFunc generate_goal_config_;
  /// For debugging the function of the function called when checking the configuration
  CheckFeasibilityCallBackFunc check_feasibility_callback_;
  /// For debugging the function of the function called when node is added
  AddNodeCallBackFunc add_node_callback_;
  /// For debugging the function of the function called at the time of START
  AddStartCallBackFunc add_start_callback_;
  /// For debugging the function of the function called when GOAL is generated
  AddGoalCallBackFunc add_goal_callback_;
  /// For debugging mainly callbacks called at ConstraintConfig
  ConstrainConfigCallBackFunc constrain_config_callback_;
};
}  // namespace tmc_rplanner

#endif  // TMC_MANIPULATION_TMC_RPLANNER_CONFIGURATION_SPACE_HPP_
