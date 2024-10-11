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
/// @brief    Common definition of robot planner

#ifndef TMC_ROBOT_RPLANNER_ROBOT_RPLANNER_COMMON_HPP_
#define TMC_ROBOT_RPLANNER_ROBOT_RPLANNER_COMMON_HPP_

#include <stdint.h>

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_rplanner/planner_common.hpp>

namespace tmc_robot_planner {

using Pose = Eigen::Affine3d;

using NameSeq = std::vector<std::string>;

using Config = Eigen::VectorXd;

// In cases where the meaning overlaps with Moveit_msgs/Msg/MoveiterRorcodes, the numbers are aligned.
enum ErrorCode {
  kSuccess = 1,
  kPlanningFailed = -1,
  kTimedOut = -6,
  kStartStateInCollision = -10,
  kGoalStateInCollision = -12,
  kNoIKSolution = -31,
  kInvalidLinkName = -18,
  kIKLinkInCollision = -33,
  kKinematicsStateInCollision = -35,
  kShortcutTimedOut = -36
};

//  end namespace tmc_robot_rplanner
}  // namespace tmc_robot_planner

#endif  // #ifndef TMC_ROBOT_RPLANNER_ROBOT_PLANNER_COMMON_HPP_
