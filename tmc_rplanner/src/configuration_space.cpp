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
/// @file     rplanner_space.cpp
/// @brief Basic operations in configuration space for planning
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2011.10.25
/// @note     [1.0.0] 2011.10.19 Newly created

#include <limits>
#include <tmc_rplanner/configuration_space.hpp>

namespace {
// Considered as no progress if it doesn't advance further
double kAdvancedEps = 1e-6;
}

namespace tmc_rplanner {

/// @func CheckTransferabilityByDividing
/// @brief Simply check by dividing the start and end points by division_num
/// @param src_config Start configuration
/// @param dst_config End configuration
/// @param check_feasibility Check if the configuration is feasible
/// @param cacl_distance Distance calculation function
/// @param sub_delta Granularity when checking transferability
/// @retval true: Transferable, false: Not transferable
/// @exception invalid_argument
/// @exception DimensionMismatch
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
    // Considered attached if extremely close
    return true;
  }

  for (double delta = 0; delta < src_to_dst_norm; delta += sub_delta) {
    if (!check_feasibility(
            src_config +
            src_to_dst * delta / src_to_dst_norm)) {
      return false;
    }
  }
  // Always check at the end
  if (!check_feasibility(dst_config)) {
    return false;
  }
  return true;
}

/// @func NewConfig
/// @brief Calculate new that has advanced delta based on distance from src to dst
///        If the distance from src to dst is less than or equal to delta, set is_reached_out to true
///        and return dst
/// @param src_config Initial configuration
/// @param dst_config Target configuration
/// @param delta Distance measured from src to approach dst
/// @param is_reached_out true: Reached, false: Not reached
/// @return Configuration that has advanced delta based on distance from src to dst
/// @exception invalid_argument
/// @exception DimensionMismatch
Config ConfigurationSpace::NewConfig(
    const Config& src_config, const Config& dst_config,
    double delta, bool& is_reached_out) const {
  // Delta must be positive
  if (delta < std::numeric_limits<double>::min()) {
    throw std::invalid_argument("Delta must be positive.");
  }
  // Configuration size must be correct
  if (dof_ != src_config.size() ||
      (dof_ != dst_config.size())) {
    throw DimensionMismatch("Configuration size mismatch.");
  }
  double length = CalcDistance(src_config, dst_config);
  // Calculate the unit vector computed by distance
  if (length > delta) {
    is_reached_out = false;
    return src_config + (dst_config-src_config)/length * delta;
  } else {
    is_reached_out = true;
    return dst_config;
  }
}


/// @func bool CheckLine
/// @brief Check linear trajectory
/// Perform transition checks at intervals of delta from start_config to goal_config.
/// Return the series if it can transition to goal_config
/// @param src_config Initial configuration
/// @param dst_config Final configuration
/// @param delta Check width
/// @param path_out Checked configuration series
/// @return true: Transferable false: Not transferable
/// @exception DimensionMismatch
bool ConfigurationSpace::CheckLine(
    const Config& src_config,
    const Config& dst_config,
    double delta,
    Path& path_out) const {
  return CheckLine(src_config, dst_config, delta,
                   TerminateConditionFunc(), path_out);
}


/// @func bool CheckLine
/// @brief Check linear trajectory
/// Perform transition checks at intervals of delta from start_config to goal_config.
/// Return the series if it can transition to goal_config
/// @param src_config Initial configuration
/// @param dst_config Final configuration
/// @param delta Check width
/// @param terminate Termination condition
/// @param path_out Checked configuration series
/// @return true: Transferable false: Not transferable
/// @exception DimensionMismatch
bool ConfigurationSpace::CheckLine(
    const Config& src_config,
    const Config& dst_config,
    double delta,
    TerminateConditionFunc terminate,
    Path& path_out) const {
  // Configuration size must be correct
  if (dof_ != src_config.size() ||
      (dof_ != dst_config.size())) {
    throw DimensionMismatch("Configuration size mismatch.");
  }

  bool reached = false;
  path_out.clear();
  path_out.push_back(src_config);
  Config point = src_config;
  Config next_point = src_config;

  // If src_config and dst_config are equal, only perform interference check with constrain_config
  // and checkfeasibility, then return the path
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
    // Considered a failure if the result of constrain config is far from the previous location or hasn't advanced
    // Considered a failure
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

/// @brief Calculate the distance between two configurations
///       If distance is set in planner_param_, calculate with it
///      Otherwise, return Euclidean distance
/// optional: distance
/// @param config1 Configuration 1
/// @param config2 Configuration 2
/// @return Distance
/// @exception DimensionMismatch
double ConfigurationSpace::CalcDistance(const Config& config1,
                                        const Config& config2) const {
  // Configuration size must be correct
  if (dof_ != config1.size() || (dof_ != config2.size())) {
    throw DimensionMismatch("Configuration size mismatch.");
  }
  if (calc_distance_) {
    return calc_distance_(config1, config2);
  } else {
    return (config1 - config2).norm();
  }
}

/// @brief Check if the configuration is valid
///        required: check_feasibility
/// @param config Configuration
/// @return true: Valid false: Invalid
/// @exception LackRequiredFunc
/// @exception DimensionMismatch
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

/// @brief Check the transferability between two configurations.
///        If check_transferability is not set in planner_param_, only check the terminal value
///
//         required: check_feasibility or check_transferability
/// @param src_config Start configuration
/// @param dst_config End configuration
/// @return true: Transferable false: Not transferable
/// @exception DimensionMismatch
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


/// @brief Generate a random configuration
///        required: random_config
/// @return Random configuration
/// @exception LackRequiredFunc
/// @exception DimensionMismatch
Config ConfigurationSpace::GenerateRandomConfig() const {
  if (!generate_random_config_) {
    throw LackRequiredFunc("function random_config is required.");
  } else {
    return generate_random_config_();
  }
}


/// @brief Evaluate the configuration
///        required: random_config
/// @return Evaluation value
/// @exception LackRequiredFunc
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

/// @brief Function to generate target configuration
///        required: generate_goal_config
/// @param config Target configuration
/// @return Success, failure
bool ConfigurationSpace::GenerateGoalConfig(Config& config) const {
  if (generate_goal_config_) {
    return generate_goal_config_(config);
  } else {
    return false;
  }
}

/// @brief Function to generate initial configuration
///        required: generate_start_config
/// @param config Initial configuration
/// @return Success, failure
bool ConfigurationSpace::GenerateStartConfig(Config& config) const {
  if (generate_start_config_) {
    return generate_start_config_(config);
  } else {
    return false;
  }
}


/// @brief Check if the configuration meets the termination condition
///        required: check_goal_config
/// @param Configuration to be judged
/// @return true: Meets termination condition false: Does not meet termination requirements
/// @exception LackRequiredFunc
/// @exception DimensionMismatch
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

/// @brief Constrain the configuration.
/// If constraint_config is not set in planner_param_, return config_in as is
/// @param config_in: Input configuration
/// @param config_out: Constrained configuration
/// @return true: Constraint failed false: Constraint succeeded
/// @exception LackRequiredFunc
/// @exception DimensionMismatch
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

/// Constrain the Start configuration.
/// If constraint_config is not set in planner_param_, return config_in as is
/// @param config_in: Input configuration
/// @param config_out: Constrained configuration
/// @return true: Constraint failed false: Constraint succeeded
/// @exception LackRequiredFunc
/// @exception DimensionMismatch
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

/// Constrain the Goal configuration.
/// If constraint_config is not set in planner_param_, return config_in as is
/// @param config_in: Input configuration
/// @param config_out: Constrained configuration
/// @return true: Constraint failed false: Constraint succeeded
/// @exception LackRequiredFunc
/// @exception DimensionMismatch
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


/// Function called during configuration check Mainly for debugging
/// @param config Checked configuration
/// @param success Check result
void ConfigurationSpace::CheckFeasibilityCallBack(
    const Config& config,  bool success) const {
  if (check_feasibility_callback_) {
    check_feasibility_callback_(config, success);
  }
}

/// Function called when adding a node Mainly for debugging
/// @param parent Parent node
/// @param child Child node
void ConfigurationSpace::AddNodeCallBack(
    const Config& parent, const Config& child) const {
  if (add_node_callback_) {
    add_node_callback_(parent, child);
  }
}

/// Function called during start generation Mainly for debugging
/// @param config Added configuration
void ConfigurationSpace::AddStartCallBack(const Config& config) const {
  if (add_start_callback_) {
    add_start_callback_(config);
  }
}

/// Function called during goal generation Mainly for debugging
/// @param config Added configuration
void ConfigurationSpace::AddGoalCallBack(const Config& config) const {
  if (add_goal_callback_) {
    add_goal_callback_(config);
  }
}

/// Callback called during ConstraintConfig Mainly for debugging
void ConfigurationSpace::ConstrainConfigCallBack(
    const Config& config_in, const Config& config_out, bool success) const {
  if (constrain_config_callback_) {
    constrain_config_callback_(config_in, config_out, success);
  }
}


// end of namespace tmc_planner
}  // namespace tmc_rplanner
