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

#include <limits>
#include <tmc_rplanner/configuration_space.hpp>

namespace {
// If you are not going any further, you will consider no progress
double kAdvancedEps = 1e-6;
}

namespace tmc_rplanner {

/// @func CheckTransferabilityByDividing
bool CheckTransferabilityByDividing(
    const Config& src_config,
    const Config& dst_config,
    const CheckFeasibilityFunc& check_feasibility,
    const DistanceFunc& calc_distance,
    double sub_delta) {
  if (sub_delta < std::numeric_limits<double>::min()) {
    throw std::invalid_argument("Invalid division number.");
  }
  if (src_config.size() != dst_config.size()) {
    throw DimensionMismatch("Invalid Config size.");
  }

  Config src_to_dst = dst_config - src_config;
  double src_to_dst_norm = 0.0;
  if (calc_distance) {
    src_to_dst_norm = calc_distance(dst_config, src_config);
  } else {
    src_to_dst_norm = src_to_dst.norm();
  }

  if (src_to_dst_norm <
      std::numeric_limits<double>::min()) {
    // If it is extremely close, it is attached
    return true;
  }

  for (double delta = 0; delta < src_to_dst_norm; delta += sub_delta) {
    if (!check_feasibility(
            src_config +
            src_to_dst * delta / src_to_dst_norm)) {
      return false;
    }
  }
  // Absolutely investigate the end
  if (!check_feasibility(dst_config)) {
    return false;
  }
  return true;
}

/// @func NewConfig
/// @brief Calculate the New that advanced Delta based on Distance standards from SRC to DST
///        If the distance from SRC to DST is below DELTA, make is_reached_out to true.
///        Return DST
Config ConfigurationSpace::NewConfig(
    const Config& src_config, const Config& dst_config,
    double delta, bool& is_reached_out) const {
  // Delta must be positive
  if (delta < std::numeric_limits<double>::min()) {
    throw std::invalid_argument("Delta must be positive.");
  }
  // Configuration size is correct
  if (dof_ != src_config.size() ||
      (dof_ != dst_config.size())) {
    throw DimensionMismatch("Configuration size mismatch.");
  }
  double length = CalcDistance(src_config, dst_config);
  // Calculate the unit vector calculated by Distance
  if (length > delta) {
    is_reached_out = false;
    return src_config + (dst_config-src_config)/length * delta;
  } else {
    is_reached_out = true;
    return dst_config;
  }
}


/// @func bool CheckLine
/// @brief Check the straight orbit
/// Check the transition from Start_config to Goal_config every Delta.
/// If you can transition to Goal_config, return that affiliate
bool ConfigurationSpace::CheckLine(
    const Config& src_config,
    const Config& dst_config,
    double delta,
    Path& path_out) const {
  return CheckLine(src_config, dst_config, delta,
                   TerminateConditionFunc(), path_out);
}


/// @func bool CheckLine
/// @brief Check the straight orbit
/// Check the transition from Start_config to Goal_config every Delta.
/// If you can transition to Goal_config, return that affiliate
bool ConfigurationSpace::CheckLine(
    const Config& src_config,
    const Config& dst_config,
    double delta,
    TerminateConditionFunc terminate,
    Path& path_out) const {
  // Configuration size is correct
  if (dof_ != src_config.size() ||
      (dof_ != dst_config.size())) {
    throw DimensionMismatch("Configuration size mismatch.");
  }

  bool reached = false;
  path_out.clear();
  path_out.push_back(src_config);
  Config point = src_config;
  Config next_point = src_config;

  // If src_config and dst_config are equal, check interference check constrain_config
  // Return the path by using only checkFEASIBLITY
  if (CalcDistance(src_config, dst_config) < kAdvancedEps)  {
    if (!ConstrainConfig(dst_config, next_point)) {
      return false;
    }
    if (!CheckTransferability(path_out.back(), next_point)) {
      return false;
    }
    path_out.push_back(next_point);
    return true;
  }

  while (!reached) {
    if (terminate && terminate()) {
      return false;
    }
    point = NewConfig(path_out.back(), dst_config, delta, reached);
    if (!ConstrainConfig(point, next_point)) {
      return false;
    }
    bool constrain_reached = false;
    next_point = NewConfig(path_out.back(),
                           next_point, delta,
                           constrain_reached);
    // If the result of Constrain Config is not far from the last time
    // Fail
    if ((CalcDistance(next_point, path_out.back()) < kAdvancedEps)
        || (CalcDistance(next_point, dst_config)) >
        CalcDistance(path_out.back(), dst_config) + kAdvancedEps) {
      return false;
    }

    if (!CheckTransferability(path_out.back(), next_point)) {
      return false;
    }
    path_out.push_back(next_point);
  }
  return true;
}

/// @brief Calculate the distance between the two configurations
///       Distance is set to Planner_Param_
///      If it is done, if it is not calculated, return the euglid distance
double ConfigurationSpace::CalcDistance(const Config& config1,
                                        const Config& config2) const {
  // Configuration size is correct
  if (dof_ != config1.size() || (dof_ != config2.size())) {
    throw DimensionMismatch("Configuration size mismatch.");
  }
  if (calc_distance_) {
    return calc_distance_(config1, config2);
  } else {
    return (config1 - config2).norm();
  }
}

/// @brief Check if configuration is valid
///        required: check_feasibility
bool ConfigurationSpace::CheckFeasibility(const Config& config) const {
  bool feasible;
  if (dof_ != config.size()) {
    throw DimensionMismatch("Configuration size mismatch.");
  }
  if (!check_feasibility_) {
    throw LackRequiredFunc("function check_feasibility is required.");
  } else {
    feasible = check_feasibility_(config);
    CheckFeasibilityCallBack(config, feasible);
    return feasible;
  }
}

/// @brief Check the transition possibility between the two configurations.
///        If there is no check_transferability in Planner_param_, just the terminal value
///        Just check
//         required: check_feasibility or check_transferability
bool ConfigurationSpace::CheckTransferability(
    const Config& src_config,
    const Config& dst_config) const {
  if ((dof_ != src_config.size()) ||
      (dof_ != dst_config.size())) {
    throw DimensionMismatch("Configuration size mismatch.");
  }
  if (!check_transferability_) {
    return CheckFeasibility(dst_config);
  } else {
    return check_transferability_(src_config, dst_config);
  }
}


/// @brief Generate random configuration
///        required: random_config
Config ConfigurationSpace::GenerateRandomConfig() const {
  if (!generate_random_config_) {
    throw LackRequiredFunc("function random_config is required.");
  } else {
    return generate_random_config_();
  }
}


/// @brief Assessment of configuration
///        required: random_config
double ConfigurationSpace::EvaluateConfig(const Config& config) const {
  if (dof_ != config.size()) {
    throw DimensionMismatch("Configuration size mismatch.");
  }

  if (evaluate_config_) {
    return evaluate_config_(config);
  } else {
    throw LackRequiredFunc("function evaluate_config is required.");
  }
}

/// @brief General configuration generation function
///        required: generate_goal_config
bool ConfigurationSpace::GenerateGoalConfig(Config& config) const {
  if (generate_goal_config_) {
    return generate_goal_config_(config);
  } else {
    return false;
  }
}

/// @brief Initial configuration generation function
///        required: generate_start_config
bool ConfigurationSpace::GenerateStartConfig(Config& config) const {
  if (generate_start_config_) {
    return generate_start_config_(config);
  } else {
    return false;
  }
}


/// @brief Check if the configuration is included in the termination conditions
///        required: check_goal_config
bool ConfigurationSpace::CheckConfigInGoal(const Config& config) const {
  if (dof_ != config.size()) {
    throw DimensionMismatch("Configuration size mismatch.");
  }
  if (check_goal_config_) {
    return check_goal_config_(config);
  } else {
    throw LackRequiredFunc("function check_goal_config is required.");
  }
}

/// @brief Restraint the configuration.
/// If there is no Constraint_config in Planner_param_, return Config_in as it is
bool ConfigurationSpace::ConstrainConfig(
    const Config& config_in, Config& config_out) const {
  bool success = false;
  if (dof_ != config_in.size()) {
    throw DimensionMismatch("Configuration size mismatch.");
  }
  if (constrain_config_) {
    success = constrain_config_(config_in, config_out);
    ConstrainConfigCallBack(config_in, config_out, success);
    return success;
  } else {
    config_out = config_in;
    return true;
  }
}

/// Restrain the START configuration.
/// If there is no Constraint_config in Planner_param_, return Config_in as it is
bool ConfigurationSpace::ConstrainStartConfig(
    const Config& config_in, Config& config_out) const {
  bool success = false;
  if (dof_ != config_in.size()) {
    throw DimensionMismatch("Configuration size mismatch.");
  }
  if (constrain_start_config_) {
    success = constrain_start_config_(config_in, config_out);
    ConstrainConfigCallBack(config_in, config_out, success);
    return success;
  } else {
    config_out = config_in;
    return true;
  }
}

/// Restraint the Goal configuration.
/// If there is no Constraint_config in Planner_param_, return Config_in as it is
bool ConfigurationSpace::ConstrainGoalConfig(
    const Config& config_in, Config& config_out) const {
  bool success = false;
  if (dof_ != config_in.size()) {
    throw DimensionMismatch("Configuration size mismatch.");
  }
  if (constrain_goal_config_) {
    success = constrain_goal_config_(config_in, config_out);
    ConstrainConfigCallBack(config_in, config_out, success);
    return success;
  } else {
    config_out = config_in;
    return true;
  }
}


/// For debugging the function of the function called when checking the configuration
void ConfigurationSpace::CheckFeasibilityCallBack(
    const Config& config,  bool success) const {
  if (check_feasibility_callback_) {
    check_feasibility_callback_(config, success);
  }
}

/// For debugging the function of the function called when node is added
void ConfigurationSpace::AddNodeCallBack(
    const Config& parent, const Config& child) const {
  if (add_node_callback_) {
    add_node_callback_(parent, child);
  }
}

/// For debugging the function called at the time of start generation
void ConfigurationSpace::AddStartCallBack(const Config& config) const {
  if (add_start_callback_) {
    add_start_callback_(config);
  }
}

/// For debugging the function of the function called when GOAL is generated
void ConfigurationSpace::AddGoalCallBack(const Config& config) const {
  if (add_goal_callback_) {
    add_goal_callback_(config);
  }
}

/// For debugging mainly callbacks called at ConstraintConfig
void ConfigurationSpace::ConstrainConfigCallBack(
    const Config& config_in, const Config& config_out, bool success) const {
  if (constrain_config_callback_) {
    constrain_config_callback_(config_in, config_out, success);
  }
}


// end of namespace tmc_planner
}  // namespace tmc_rplanner
