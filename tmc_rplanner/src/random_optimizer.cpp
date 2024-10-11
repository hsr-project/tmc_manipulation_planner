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
/// @brief    Implementation of Pointopointplanner by Random_optimizer

#include <tmc_rplanner/random_optimizer.hpp>

namespace tmc_rplanner {

/// @brief Crease a random configuration to find the optimal solution
bool RandomOptimizer::Optimize(Config& config_out, double& value_out) {
  double max_value = 0.0;
  int32_t num_eval = 0;
  Config max_config;

  for (int32_t i = 0; i < max_itr_; ++i)  {
    // Random configuration occurs
    Config new_config = space_->GenerateRandomConfig();
    /// Check if configuration is valid
    if (space_->CheckFeasibility(new_config)) {
      /// Assessment of configuration
      ++num_eval;
      double value = space_->EvaluateConfig(new_config);
      // Maximum value update
      if (value > max_value) {
        max_value = value;
        max_config = new_config;
      }
    }
    // Ends when num_eval exceeds max_eval
    if (num_eval > max_eval_) {
      break;
    }
    // End condition check
    if (is_terminate_ && is_terminate_()) {
      break;
    }
  }
  // Failure if the number of evaluations is 0
  if (num_eval == 0) {
    return false;
  } else {
    config_out = max_config;
    value_out = max_value;
    return true;
  }
}

//  close namespace tmc_rplanner
}  // namespace tmc_rplanner
