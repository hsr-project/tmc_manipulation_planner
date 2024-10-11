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

#include <string>
#include <vector>

#include <tmc_eigen_utils/eigen_utils.hpp>

#include <tmc_robot_planner/task_space_region.hpp>

using tmc_manipulation_types::RegionValues;
using tmc_manipulation_types::TaskSpaceRegion;

namespace tmc_robot_planner {

/// Convert RegionValues ​​to Pose
/// @param region_vals X, Y, Z, Roll, Pitch, Yaw value
Pose RegionValuesToPose(const RegionValues& region_vals) {
  Eigen::Vector3d pos = region_vals.segment<3>(0);
  Eigen::Vector3d rpy = region_vals.segment<3>(3);

  return Eigen::Translation3d(pos) * tmc_eigen_utils::RPYToQuaternion(rpy);
}

/// Convert Pose to RegionValues
RegionValues PoseToRegionValues(const Pose& pose) {
  Eigen::Vector3d pos = pose.translation();
  Eigen::Vector3d rpy = tmc_eigen_utils::QuaternionToRPY(
      Eigen::Quaterniond(pose.linear()));
  RegionValues region_val;
  region_val << pos, rpy;
  return region_val;
}

/// Returns Frame, the closest to the argument in TSR
Pose CalcClosestPose(const TaskSpaceRegion& tsr,
                     const Pose& origin_to_sample) {
  Pose origin_to_sample_dash = origin_to_sample * tsr.tsr_to_end.inverse();
  Pose tsr_to_sample_dash = tsr.origin_to_tsr.inverse() * origin_to_sample_dash;
  RegionValues sample_dash = PoseToRegionValues(tsr_to_sample_dash);
  // Calculate the distance between Sample and TSR
  RegionValues distance = CalcDistanceToTsr(tsr, origin_to_sample);
  return tsr.origin_to_tsr * RegionValuesToPose(sample_dash - distance)
      * tsr.tsr_to_end;
}

/// Sampling one random from inside TSR
Pose GenerateSample(const TaskSpaceRegion& tsr) {
  RegionValues random = (tsr.max_bounds - tsr.min_bounds).array() *
      RegionValues::Random().array().abs() + tsr.min_bounds.array();
  return tsr.origin_to_tsr * RegionValuesToPose(random) * tsr.tsr_to_end;
}

/// Calculate the distance from the given Sample (coordinates of Endeffector) to TSR
RegionValues CalcDistanceToTsr(
    const TaskSpaceRegion& tsr,
    const Pose& origin_to_sample) {
  // Sample value without offset as seen from Origin
  Pose origin_to_sample_dash = origin_to_sample * tsr.tsr_to_end.inverse();
  Pose tsr_to_sample_dash = tsr.origin_to_tsr.inverse() * origin_to_sample_dash;
  Eigen::Vector3d disp_pos = tsr_to_sample_dash.translation();
  Eigen::Vector3d disp_rot = tmc_eigen_utils::QuaternionToRPY(
      Eigen::Quaterniond(tsr_to_sample_dash.linear()));
  Eigen::Vector3d distance_pos;
  // The position is simply compared to Bounds
  for (int32_t i = 0; i < 3; ++i) {
    if (disp_pos(i) < tsr.min_bounds(i)) {
      distance_pos(i) = disp_pos(i) - tsr.min_bounds(i);
    } else if (disp_pos(i) > tsr.max_bounds(i)) {
      distance_pos(i) = disp_pos(i) - tsr.max_bounds(i);
    } else {
      distance_pos(i) = 0.0;
    }
  }
  // RPY is compared with the expression of 8 patterns
  Eigen::Vector3d temp_distance_rot;
  for (int32_t i = 0; i < 3; ++i) {
    if (disp_rot(i) < tsr.min_bounds(i + 3)) {
      temp_distance_rot(i) = disp_rot(i) - tsr.min_bounds(i + 3);
    } else if (disp_rot(i) > tsr.max_bounds(i + 3)) {
      temp_distance_rot(i) = disp_rot(i) - tsr.max_bounds(i + 3);
    } else {
      temp_distance_rot(i) = 0.0;
    }
  }
  Eigen::Vector3d distance_rot = temp_distance_rot;
  double min_norm = distance_rot.norm();
  std::vector<Eigen::Vector3d> rpy_offsets(8);
  rpy_offsets[0] << M_PI, M_PI, M_PI;
  rpy_offsets[1] << M_PI, M_PI, -M_PI;
  rpy_offsets[2] << M_PI, -M_PI, M_PI;
  rpy_offsets[3] << M_PI, -M_PI, -M_PI;
  rpy_offsets[4] << -M_PI, M_PI, M_PI;
  rpy_offsets[5] << -M_PI, M_PI, -M_PI;
  rpy_offsets[6] << -M_PI, -M_PI, M_PI;
  rpy_offsets[7] << -M_PI, -M_PI, -M_PI;
  for (int32_t i = 0; i < 8; ++i) {
    double norm = (temp_distance_rot + rpy_offsets[i]).norm();
    if (norm < min_norm) {
      min_norm = norm;
      distance_rot = (temp_distance_rot + rpy_offsets[i]);
    }
  }
  RegionValues distance;
  distance << distance_pos, distance_rot;
  return distance;
}

// end of tmc_robot_rplanner
}  // namespace tmc_robot_planner


