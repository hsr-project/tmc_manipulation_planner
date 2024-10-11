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
/// @brief    Configuration optimization interface

#ifndef TMC_MANIPULATION_TMC_RPLANNER_CONFIG_OPTIMIZER_HPP_
#define TMC_MANIPULATION_TMC_RPLANNER_CONFIG_OPTIMIZER_HPP_

#include <memory>
#include <tmc_rplanner/planner_common.hpp>

namespace tmc_rplanner {

/// @class IConfigOoptimizer
/// @brief Optimization interface
class  IConfigOptimizer {
 public:
  using Ptr = std::shared_ptr<IConfigOptimizer>;
  using ConstPtr = std::shared_ptr<const IConfigOptimizer>;

  virtual ~IConfigOptimizer() {}
  /// Configuration optimization
  /// @param config_out Optimal configuration
  /// @param value_out Evaluation value
  /// @return true: Successful False: Failure
  virtual bool Optimize(Config& config_out, double& value_out) = 0;
};
}  // namespace tmc_rplanner

#endif  // #define TMC_MANIPULATION_TMC_RPLANNER_CONFIG_OPTIMIZER_HPP_
