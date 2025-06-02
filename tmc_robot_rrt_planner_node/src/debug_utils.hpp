/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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
///
/// debug_utils.hpp - debug utilities for planner
///
///

#ifndef TMC_ROBOT_RRT_PLANNER_NODE_DEBUG_UTILS_HPP_
#define TMC_ROBOT_RRT_PLANNER_NODE_DEBUG_UTILS_HPP_

#include <cstdlib>
#include <string>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tmc_eigen_bridge/eigen_bridge.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_robot_collision_detector/robot_collision_detector.hpp>
#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>
#include <tmc_robot_planner/robot_cbirrt_planner.hpp>

namespace tmc_robot_rrt_planner_node {

/// Convert CollisionObject to Marker
/// @param [in] object Object of type CollisionObject
/// @param [in] origin_to_object Position and orientation of the object from the reference coordinate
/// @param [in] origin_to_robot Position and orientation of the robot from the reference coordinate
visualization_msgs::MarkerArray ObjectToMarker(
    const std::vector<tmc_manipulation_msgs::CollisionObject>& objects,
    const std::vector<geometry_msgs::Pose>& origin_to_object,
    const geometry_msgs::Pose& origin_to_robot);

visualization_msgs::MarkerArray MapToMarker(
    const tmc_mapping_msgs::CollisionMap& collision_map,
    const geometry_msgs::Pose& origin_to_map);

void PublishEnvironmentDebug(
    const tmc_manipulation_msgs::CollisionEnvironment& environment,
    bool delete_before_publish,
    ros::Publisher& marker_pub);

void PublishEnvironmentDebug(
    const tmc_manipulation_msgs::CollisionEnvironment& environment,
    ros::Publisher& marker_pub);

void FetchCollisionEnvironment(
    const tmc_manipulation_types::JointState& joint_state,
    const Eigen::Affine3d& origin_to_base,
    const tmc_manipulation_types::AttachedObjectSeq& attached_objects,
    const tmc_robot_collision_detector::RobotCollisionDetector::Ptr&
    robot_collision_detector,
    const tmc_manipulation_msgs::CollisionEnvironment& environment,
    tmc_manipulation_msgs::CollisionEnvironment& environment_out);

}  // namespace tmc_robot_rrt_planner_node

#endif  // TMC_ROBOT_RRT_PLANNER_NODE_DEBUG_UTILS_HPP_
