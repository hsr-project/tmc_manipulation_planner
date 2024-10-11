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
/// @brief    TSR(Task Space Region)

#ifndef TMC_ROBOT_RPLANNER_TASK_SPACE_REGION_HPP_
#define TMC_ROBOT_RPLANNER_TASK_SPACE_REGION_HPP_

#include <string>
#include <vector>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_robot_planner/robot_planner_common.hpp>
#include <tmc_rplanner/planner_common.hpp>

namespace tmc_robot_planner {

Pose CalcClosestPose(const tmc_manipulation_types::TaskSpaceRegion& tsr,
                     const Pose& origin_to_frame);
Pose GenerateSample(const tmc_manipulation_types::TaskSpaceRegion& tsr);
tmc_manipulation_types::RegionValues CalcDisplacementToTsr(
    const tmc_manipulation_types::TaskSpaceRegion& tsr,
    const Pose& origin_to_frame);
tmc_manipulation_types::RegionValues CalcDistanceToTsr(
    const tmc_manipulation_types::TaskSpaceRegion& tsr,
    const Pose& origin_to_frame);

// end of tmc_robot_rplanner
}  // namespace tmc_robot_planner

#endif
