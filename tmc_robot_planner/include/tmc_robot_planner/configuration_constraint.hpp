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

#ifndef TMC_ROBOT_PLANNER_CONFIGURATION_CONSTRAINT_HPP_
#define TMC_ROBOT_PLANNER_CONFIGURATION_CONSTRAINT_HPP_

#include <memory>
#include <string>
#include <vector>

#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>
#include <tmc_rplanner/planner_common.hpp>

namespace tmc_robot_planner {

class IConfigurationConstraint {
 public:
  using Ptr = std::shared_ptr<IConfigurationConstraint>;
  using ConstPtr = std::shared_ptr<const IConfigurationConstraint>;

  /// Default constructor
  IConfigurationConstraint() {}
  /// Destructor
  virtual ~IConfigurationConstraint() {}
  /// Constraint function
  virtual bool Constrain(
      const std::vector<std::string>& use_joints,
      const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
      const tmc_rplanner::Config& config_in,
      tmc_rplanner::Config& config_out) = 0;
  /// Reset internal states
  /// This function is called every planning service
  /// @return  bool  return false if failure
  /// @note  this method should be pure virtual function
  virtual bool Reset() {
    return true;
  }

 private:
  /// Copy constructor
  IConfigurationConstraint(const IConfigurationConstraint& rhs);
  /// Assignment operator
  IConfigurationConstraint& operator=(const IConfigurationConstraint& rhs);
};

}  // namespace tmc_robot_planner

#endif  // TMC_ROBOT_PLANNER_CONFIGURATION_CONSTRAINT_HPP_
