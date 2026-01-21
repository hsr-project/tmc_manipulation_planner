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
/// @file     random_optimizer.hpp
/// @brief Optimization using the Monte Carlo method
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2011.10.25
/// @note     [1.0.0] 2011.10.19 Newly created

#ifndef TMC_MANIPULATION_TMC_RPLANNER_RANDOM_OPTIMIZER_HPP_
#define TMC_MANIPULATION_TMC_RPLANNER_RANDOM_OPTIMIZER_HPP_

#include <tmc_rplanner/config_optimizer.hpp>
#include <tmc_rplanner/configuration_space.hpp>

namespace tmc_rplanner {

/// @class Random_Optimizer
/// @brief Implementation by ConfigOptimizer's random_optimizer
/// @note Only returns the optimal value among random configurations
class RandomOptimizer : public IConfigOptimizer {
 public:
  /// @brief Pass the planner space and termination conditions
  /// @param space Pointer to the planner space
  /// @param max_itr Maximum number of iterations
  /// @param max_eval Maximum number of evaluations
  /// @param is_terminate Forced termination condition
  RandomOptimizer(ConfigurationSpace::Ptr space,
                  int32_t max_itr,
                  int32_t max_eval,
                  TerminateConditionFunc is_terminate) :
      space_(space), max_itr_(max_itr),
      max_eval_(max_eval), is_terminate_(is_terminate) {}

  /// @brief Pass the planner space and termination conditions
  /// @param space Pointer to the planner space
  /// @param max_itr Maximum number of iterations
  /// @param max_eval Maximum number of evaluations
  RandomOptimizer(ConfigurationSpace::Ptr space,
                  int32_t max_itr,
                  int32_t max_eval) :
      space_(space), max_itr_(max_itr),
      max_eval_(max_eval) {}
  virtual ~RandomOptimizer() {}
  /// Execute optimization
  virtual bool Optimize(Config& config_out,
                        double& value_out);

 private:
  // Prohibition of copying
  RandomOptimizer(const RandomOptimizer&);
  RandomOptimizer& operator=(const RandomOptimizer&);
  const ConfigurationSpace::Ptr space_;
  /// Maximum number of iterations
  const int32_t max_itr_;
  /// Maximum number of evaluations
  const int32_t max_eval_;
  TerminateConditionFunc is_terminate_;
};
}  // namespace tmc_rplanner

#endif  // TMC_MANIPULATION_TMC_RPLANNER_RANDOM_OPTIMIZER_HPP_
